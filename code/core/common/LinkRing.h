#pragma once
#include "common.h"
#include <stdbool.h>

// Lock-free single-producer / single-consumer byte rings shared between ARM7 (USB CDC) and
// ARM9 (SIO emulation). C-compatible so it can be used from both C and C++ sources.
//
// Placement: the last 4K of the ARM9 core's ewram linker region, 0x021FF000 - 0x02200000, is
// carved out in core/arm9/gbarunner9.ld for these rings. Do NOT move them into 0x02200000+:
// that is the linear GBA ROM region (MemoryEmulator/RomDefs.h), which the ARM9 fills with the
// first 2MB of the ROM after the ARM7 has initialised the rings.
//
// Coherency: the ARM7 has no cache, but the ARM9 MPU marks all of 0x02000000 - 0x023FFFFF as
// data-cacheable (MemoryProtectionConfiguration.cpp). The ARM9 must therefore only access the
// rings through linkRxRing()/linkTxRing(), which return an uncached alias on ARM9. With that,
// volatile loads/stores are sufficient for SPSC between the two cores.

#define LINK_RING_SIZE 1024 // power of two
#define LINK_RING_MASK (LINK_RING_SIZE - 1)

typedef struct
{
	volatile u8  data[LINK_RING_SIZE];
	volatile u32 head; // written by producer only
	volatile u32 tail; // written by consumer only
} LinkRing;

#define LINK_RX_RING_ADDR 0x021FF000 // PC -> DS (CDC RX -> SIO RX)
#define LINK_TX_RING_ADDR 0x021FF800 // DS -> PC (SIO TX -> CDC TX)

#ifdef __cplusplus
static_assert(sizeof(LinkRing) <= 0x800, "two rings must fit the 4K reservation");
#else
_Static_assert(sizeof(LinkRing) <= 0x800, "two rings must fit the 4K reservation");
#endif

static inline u32 linkRing_accessAddr(u32 addr)
{
#ifdef ARM9
	// Uncached alias of main memory (covered only by MPU region 0, which is uncached).
	// NTR mode: 4MB main RAM is mirrored at 0x02400000.
	// TWL mode: 0x02400000 is real RAM, so use the 0x0C000000 main-RAM mirror instead.
	// (Same DSi-mode test as Environment::Initialize.)
	if (((*(vu32*)0x04004000) & 3) == 1)
		return addr - 0x02000000 + 0x0C000000;
	return addr + 0x00400000;
#else
	return addr;
#endif
}

static inline LinkRing* linkRxRing(void)
{
	return (LinkRing*)linkRing_accessAddr(LINK_RX_RING_ADDR);
}

static inline LinkRing* linkTxRing(void)
{
	return (LinkRing*)linkRing_accessAddr(LINK_TX_RING_ADDR);
}

// Reset a ring to empty. Only safe while neither side is using it (startup).
static inline void linkRing_reset(LinkRing* r)
{
	r->head = 0;
	r->tail = 0;
}

// Producer: true if a push would fail.
static inline bool linkRing_isFull(const LinkRing* r)
{
	return (((r->head & LINK_RING_MASK) + 1) & LINK_RING_MASK) == (r->tail & LINK_RING_MASK);
}

// Producer: push one byte. Returns false if the ring is full.
static inline bool linkRing_push(LinkRing* r, u8 b)
{
	u32 h = r->head & LINK_RING_MASK;
	u32 next = (h + 1) & LINK_RING_MASK;
	if (next == (r->tail & LINK_RING_MASK))
	{
		return false; // full
	}
	r->data[h] = b;
	r->head = next;
	return true;
}

// Consumer: pop one byte into *b. Returns false if the ring is empty.
static inline bool linkRing_pop(LinkRing* r, u8* b)
{
	u32 t = r->tail & LINK_RING_MASK;
	if (t == (r->head & LINK_RING_MASK))
	{
		return false; // empty
	}
	*b = r->data[t];
	r->tail = (t + 1) & LINK_RING_MASK;
	return true;
}

// Either side: number of bytes currently in the ring.
static inline u32 linkRing_count(const LinkRing* r)
{
	return ((r->head & LINK_RING_MASK) - (r->tail & LINK_RING_MASK)) & LINK_RING_MASK;
}

// Producer: number of bytes that can still be pushed.
static inline u32 linkRing_free(const LinkRing* r)
{
	return (LINK_RING_SIZE - 1) - linkRing_count(r);
}

// Consumer: read the byte `offset` positions after the tail without consuming it.
// Only valid for offset < linkRing_count(r).
static inline u8 linkRing_peek(const LinkRing* r, u32 offset)
{
	return r->data[((r->tail & LINK_RING_MASK) + offset) & LINK_RING_MASK];
}

// Producer: write n bytes all-or-nothing, publishing them with a single head update so the consumer
// never sees a partial block. Returns false (and writes nothing) if there isn't room.
static inline bool linkRing_writeBlock(LinkRing* r, const u8* data, u32 n)
{
	if (linkRing_free(r) < n)
	{
		return false;
	}
	u32 h = r->head & LINK_RING_MASK;
	for (u32 i = 0; i < n; i++)
	{
		r->data[(h + i) & LINK_RING_MASK] = data[i];
	}
	r->head = (h + n) & LINK_RING_MASK;
	return true;
}
