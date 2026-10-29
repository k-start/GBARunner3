// Size matters more than speed here: this file's code lives in VRAM A together with the rest of the
// ARM9 core's .text/.bss, which is full, and none of it is hot (at most 9 transfers per frame plus
// the VBlank work), so it is built with -Os.
// NEVER compile GBARunner3 core code as Thumb: crt0 sets CP15 control bit 15 (L4, ARMv4T
// compatibility, needed for GBA code), which makes ldr pc / pop {pc} NOT switch ARM<->Thumb. The
// linker's ARM->Thumb stubs (ldr pc, =func|1) and Thumb returns then crash on hardware. melonDS doesn't
// model L4, so it hides this (Plan.md, 3DS regression 2026-10-29).
#pragma GCC optimize("Os")

// WARNING: plain .bss/.data of the ARM9 core lives in VRAM A, and VRAM ignores 8-bit writes. Every
// non-ewram static in this file must be u16 or u32, never u8 or bool. (Byte buffers are fine in
// [[gnu::section(".ewram.bss")]], which is main RAM.)

#include "common.h"
#include "LinkProtocol.h"
#include "LinkSio.h"
#include "Emulator/IoRegisters.h"

// TX-ring producers on the ARM9: linkSio_init (before IRQs are enabled) and linkSio_vblank (VBlank
// IRQ). They can't run concurrently. When Phase 2a adds producers outside the IRQ (SIO register
// store handlers), those must disable IRQs around linkSio_send.

static u32 sFrame;
static u32 sRxPackets;
static u32 sRxCrcErrors;
static u32 sRxResyncBytes;
static u32 sTxDrops;

static void linkSio_send(u8 type, const void* payload, u32 len);

// ---- SIO observation log (Phase 2a) ----
// Large buffers go in .ewram.bss: the default .bss is in VRAM A, which is full.
// Producer: linkSio_logStore/linkSio_logLoad, called from the guest IO store/load handlers
// (Peripherals/Sio.s). Consumer: linkSio_vblank (VBlank IRQ), which packs records into 'L'
// packets. Same core, single producer / single consumer, so ordered index updates suffice.
// (Assumes the guest never DMAs into the SIO registers, which would make the VBlank-IRQ DMA path a
// second producer.)
#define LINK_LOG_SIZE 256 // records, power of two (ewram is tight: see the heap note in Plan.md)
[[gnu::section(".ewram.bss")]]
static LinkSioLogRecord sLog[LINK_LOG_SIZE];
static volatile u32 sLogHead;
static volatile u32 sLogTail;
static u32 sLogRecords;
static u32 sLogDrops;
static u16 sLastLoad[0x40 / 2];   // last logged load value per register in 0x100-0x13F
static u32 sLastLoadValid;        // bit per sLastLoad entry
[[gnu::section(".ewram.bss")]]
static u8 sLogPayload[LINK_SIO_LOG_PER_PKT * sizeof(LinkSioLogRecord)];

static volatile u32 sLogEnabled = 1; // LINK_OPT_SIO_LOG, set by the PC ('O' packet)

static void linkSio_logRecord(u32 reg, u32 flags, u32 value)
{
	if (!sLogEnabled)
	{
		return;
	}
	u32 head = sLogHead;
	if (((head + 1) & (LINK_LOG_SIZE - 1)) == sLogTail)
	{
		sLogDrops++;
		return;
	}
	LinkSioLogRecord* rec = &sLog[head];
	rec->frame = (u16)sFrame;
	rec->vcount = *(vu16*)0x04000006; // DS REG_VCOUNT
	rec->reg = (u8)reg;
	rec->flags = (u8)flags;
	rec->value = (u16)value;
	__asm__ volatile("" ::: "memory"); // record before head (the VBlank IRQ may read it)
	sLogHead = (head + 1) & (LINK_LOG_SIZE - 1);
	sLogRecords++;
}

static void linkSio_logAccess(u32 addr, u32 value, u32 width, u32 flags)
{
	u32 reg = (addr - 0x04000100) & 0xFF;
	if (width == 4)
	{
		linkSio_logRecord(reg, flags | LINK_SIO_LOG_FLAG_32, value & 0xFFFF);
		linkSio_logRecord(reg + 2, flags | LINK_SIO_LOG_FLAG_32, value >> 16);
	}
	else
	{
		linkSio_logRecord(reg, flags, value & 0xFFFF);
	}
}

// Called from Peripherals/Sio.s for every guest store to a link-related register.
void linkSio_logStore(u32 addr, u32 value, u32 width)
{
	linkSio_logAccess(addr, value, width, 0);
}

// Called from Peripherals/Sio.s for every guest load of a link-related register. Only changes are
// logged, since games poll SIOCNT in tight loops.
void linkSio_logLoad(u32 addr, u32 value, u32 width)
{
	u32 reg = (addr - 0x04000100) & 0xFF;
	u32 count = width == 4 ? 2 : 1;
	bool changed = false;
	for (u32 i = 0; i < count; i++)
	{
		u32 idx = ((reg >> 1) + i) & ((0x40 / 2) - 1);
		u16 v = (u16)(value >> (16 * i));
		if (!(sLastLoadValid & (1u << idx)) || sLastLoad[idx] != v)
		{
			changed = true;
			sLastLoad[idx] = v;
			sLastLoadValid |= 1u << idx;
		}
	}
	if (changed)
	{
		linkSio_logAccess(addr, value, width, LINK_SIO_LOG_FLAG_LOAD);
	}
}

// Move buffered log records into 'L' packets while the TX ring has room; anything that doesn't fit
// stays buffered for the next frame.
static void linkSio_flushLog(void)
{
	while (true)
	{
		u32 tail = sLogTail;
		u32 avail = (sLogHead - tail) & (LINK_LOG_SIZE - 1);
		if (avail == 0)
		{
			break;
		}
		u32 n = avail > LINK_SIO_LOG_PER_PKT ? LINK_SIO_LOG_PER_PKT : avail;
		u32 len = n * sizeof(LinkSioLogRecord);
		if (linkRing_free(linkTxRing()) < len + LINK_PKT_OVERHEAD)
		{
			break;
		}
		for (u32 i = 0; i < n; i++)
		{
			const u8* src = (const u8*)&sLog[(tail + i) & (LINK_LOG_SIZE - 1)];
			for (u32 b = 0; b < sizeof(LinkSioLogRecord); b++)
			{
				sLogPayload[i * sizeof(LinkSioLogRecord) + b] = src[b];
			}
		}
		sLogTail = (tail + n) & (LINK_LOG_SIZE - 1);
		linkSio_send(LINK_PKT_SIO_LOG, sLogPayload, len);
	}
}

// ---- Emulated link-cable partner (Phase 2b, Plan.md section 5.1a) ----
// The DS game is the link.c master. Every transfer it starts completes immediately, and the DS
// itself supplies the child's word, so both "GBAs" see identical SIOMULTI tuples (link.c checksums
// every burst):
//   handshake       child sends SLAVE_HANDSHAKE until the master sends MASTER_HANDSHAKE
//   post-handshake  one more SLAVE_HANDSHAKE (the real child pre-loaded it); this transfer is slot 0
//                   of the first burst
//   data phase      bursts of 9 transfers: slot 0 = checksum (the child's checksum equals the
//                   master's, because the tuples are identical, so mirror the master's word),
//                   slots 1-8 = one 8-word child command from the 'C' queue, or zeros (no command)
//   SIOMULTI2/3     always 0xFFFF (absent players), so the game sees exactly 2 players
// Only whole commands cross USB: 'M' (master -> PC) and 'C' (PC -> child queue).
//
// Threading: linkSio_multiCntStore runs in the guest's IO store handler. The VBlank IRQ
// (linkSio_vblank) can interrupt it but not the other way round. Shared state is single-producer /
// single-consumer queues plus flags that only one side writes.

#define SIOCNT_SI          (1 << 2)
#define SIOCNT_SD          (1 << 3)
#define SIOCNT_START       (1 << 7)
#define SIOCNT_IRQ         (1 << 14)
#define SIOCNT_STATUS_BITS 0x7C // SI, SD, ID (2 bits), error

#define SLAVE_HANDSHAKE  0xB9A0
#define MASTER_HANDSHAKE 0x8FFF

#define LINK_PHASE_HANDSHAKE 0
#define LINK_PHASE_POST      1
#define LINK_PHASE_DATA      2

static volatile u32 sLinkMode = LINK_MODE_STOCK; // written by VBlank ('O'), read by the store handler
static volatile u32 sLinkResetRequested;         // set by VBlank, applied by the store handler
static u32 sLinkPhase;
static u32 sBurstSlot;
static u32 sTransfers;
static u32 sBursts;
static u32 sMasterCmds;
static u32 sChildCmds;
static u32 sLinkDrops;
static u16 sMasterWords[8];
static u16 sChildWords[8];

// PC -> child commands. Producer: VBlank ('C' packets). Consumer: store handler.
#define LINK_CHILDQ_SIZE 32 // power of two; back-pressured, so it needn't match link.c's 50
[[gnu::section(".ewram.bss")]]
static u16 sChildQ[LINK_CHILDQ_SIZE][8];
static volatile u32 sChildQHead;
static volatile u32 sChildQTail;

// Link-layer output ('M' commands, 'E' events). Producer: store handler. Consumer: VBlank, which
// is the only writer of the TX ring.
typedef struct
{
	u8 type;
	u8 len;
	u8 payload[sizeof(LinkCommandPayload) > sizeof(LinkEventPayload) ? sizeof(LinkCommandPayload) : sizeof(LinkEventPayload)];
} LinkOutRecord;
#define LINK_OUTQ_SIZE 32 // power of two; flushed every frame (max ~1 'M' per burst)
[[gnu::section(".ewram.bss")]]
static LinkOutRecord sOutQ[LINK_OUTQ_SIZE];
static volatile u32 sOutQHead;
static volatile u32 sOutQTail;

static void linkSio_outPush(u8 type, const void* payload, u32 len)
{
	u32 head = sOutQHead;
	if (((head + 1) & (LINK_OUTQ_SIZE - 1)) == sOutQTail)
	{
		sLinkDrops++;
		return;
	}
	LinkOutRecord* rec = &sOutQ[head];
	rec->type = type;
	rec->len = (u8)len;
	for (u32 i = 0; i < len; i++)
	{
		rec->payload[i] = ((const u8*)payload)[i];
	}
	__asm__ volatile("" ::: "memory");
	sOutQHead = (head + 1) & (LINK_OUTQ_SIZE - 1);
}

static void linkSio_linkEvent(u8 code, u8 arg)
{
	LinkEventPayload ev;
	ev.code = code;
	ev.arg = arg;
	ev.reserved = 0;
	ev.frame = sFrame;
	ev.transfers = sTransfers;
	linkSio_outPush(LINK_PKT_EVENT, &ev, sizeof(ev));
}

static void linkSio_linkReset(void)
{
	sLinkPhase = LINK_PHASE_HANDSHAKE;
	sBurstSlot = 0;
	linkSio_linkEvent(LINK_EVENT_RESET, 0);
}

// Take the next child command for the burst that is starting, or zeros ("no command").
static void linkSio_loadChildCommand(void)
{
	u32 tail = sChildQTail;
	if (tail == sChildQHead)
	{
		for (int i = 0; i < 8; i++)
		{
			sChildWords[i] = 0;
		}
		return;
	}
	for (int i = 0; i < 8; i++)
	{
		sChildWords[i] = sChildQ[tail][i];
	}
	__asm__ volatile("" ::: "memory");
	sChildQTail = (tail + 1) & (LINK_CHILDQ_SIZE - 1);
	sChildCmds++;
}

static void linkSio_burstDone(void)
{
	sBursts++;
	u16 any = 0;
	for (int i = 0; i < 8; i++)
	{
		any |= sMasterWords[i];
	}
	if (!any)
	{
		return; // all-zero = "no command" (link.c doesn't queue those either)
	}
	LinkCommandPayload cmd;
	cmd.burst = sBursts;
	for (int i = 0; i < 8; i++)
	{
		cmd.words[i] = sMasterWords[i];
	}
	sMasterCmds++;
	linkSio_outPush(LINK_PKT_MASTER_CMD, &cmd, sizeof(cmd));
}

// One multiplayer transfer: compute the child's word and latch all four SIOMULTI registers.
static void linkSio_linkTransfer(void)
{
	u16* io = (u16*)emu_ioRegisters;
	u16 masterWord = io[0x12A / 2]; // SIOMLT_SEND
	u16 childWord;
	switch (sLinkPhase)
	{
		case LINK_PHASE_HANDSHAKE:
		{
			childWord = SLAVE_HANDSHAKE;
			if (masterWord == MASTER_HANDSHAKE)
			{
				sLinkPhase = LINK_PHASE_POST;
				linkSio_linkEvent(LINK_EVENT_HANDSHAKE, 0);
			}
			break;
		}
		case LINK_PHASE_POST:
		{
			childWord = SLAVE_HANDSHAKE;
			sLinkPhase = LINK_PHASE_DATA;
			sBurstSlot = 1;
			linkSio_loadChildCommand();
			linkSio_linkEvent(LINK_EVENT_DATA_START, 0);
			break;
		}
		default:
		{
			if (sBurstSlot == 0)
			{
				childWord = masterWord; // checksum slot
				linkSio_loadChildCommand();
			}
			else
			{
				childWord = sChildWords[sBurstSlot - 1];
				sMasterWords[sBurstSlot - 1] = masterWord;
			}
			if (++sBurstSlot == 9)
			{
				sBurstSlot = 0;
				linkSio_burstDone();
			}
			break;
		}
	}
	io[0x120 / 2] = masterWord; // SIOMULTI0: our own word comes back in our slot
	io[0x122 / 2] = childWord;  // SIOMULTI1
	io[0x124 / 2] = 0xFFFF;     // SIOMULTI2: absent
	io[0x126 / 2] = 0xFFFF;     // SIOMULTI3: absent
	sTransfers++;
}

// Called from Peripherals/Sio.s for every guest SIOCNT store in multiplayer mode. Returns the
// value to store, with bit 31 set if the SIO IRQ must be raised.
u32 linkSio_multiCntStore(u32 value)
{
	if (sLinkMode != LINK_MODE_PARTNER)
	{
		// Stock GBARunner3: "child, all ready"; START stays set, so transfers never complete.
		return (value & ~SIOCNT_STATUS_BITS) | SIOCNT_SI | SIOCNT_SD;
	}
	if (sLinkResetRequested || !(value & SIOCNT_IRQ))
	{
		// link.c's EnableSerial/DisableSerial write SIOCNT without the IRQ bit; in-session writes
		// (START) always keep it, so this marks a (re)open of the link.
		sLinkResetRequested = 0;
		linkSio_linkReset();
	}
	value = (value & ~SIOCNT_STATUS_BITS) | SIOCNT_SD; // master: SI=0, SD=1, ID=0, no error
	if (value & SIOCNT_START)
	{
		linkSio_linkTransfer();
		value &= ~SIOCNT_START;
		if (value & SIOCNT_IRQ)
		{
			value |= 1u << 31;
		}
	}
	return value;
}

// VBlank: move queued link output into packets while the TX ring has room (never dropped here).
static void linkSio_flushLinkOut(void)
{
	while (sOutQTail != sOutQHead)
	{
		const LinkOutRecord* rec = &sOutQ[sOutQTail];
		if (linkRing_free(linkTxRing()) < rec->len + LINK_PKT_OVERHEAD)
		{
			break;
		}
		linkSio_send(rec->type, rec->payload, rec->len);
		sOutQTail = (sOutQTail + 1) & (LINK_OUTQ_SIZE - 1);
	}
}

[[gnu::section(".ewram.bss")]]
static u8 sRxPkt[LINK_PKT_MAX_SIZE];
[[gnu::section(".ewram.bss")]]
static u8 sTxPkt[LINK_PKT_MAX_SIZE];

static void linkSio_send(u8 type, const void* payload, u32 len)
{
	u32 size = linkPkt_build(sTxPkt, type, payload, len);
	if (!linkRing_writeBlock(linkTxRing(), sTxPkt, size))
	{
		sTxDrops++;
	}
}

static void linkSio_sendHello(void);

static void linkSio_handlePacket(u8 type, const u8* payload, u32 len)
{
	switch (type)
	{
		case LINK_PKT_PING:
		{
			linkSio_send(LINK_PKT_PONG, payload, len);
			break;
		}
		case LINK_PKT_HELLO_REQ:
		{
			linkSio_sendHello();
			break;
		}
		case LINK_PKT_OPTIONS:
		{
			if (len >= 2)
			{
				const LinkOptionsPayload* opt = (const LinkOptionsPayload*)payload;
				sLogEnabled = (opt->flags & LINK_OPT_SIO_LOG) ? 1 : 0;
				if (opt->mode != sLinkMode)
				{
					sLinkMode = opt->mode == LINK_MODE_PARTNER ? LINK_MODE_PARTNER : LINK_MODE_STOCK;
					sLinkResetRequested = 1;
				}
				// Always confirm (direct send: VBlank is the TX-ring producer).
				LinkEventPayload ev = { LINK_EVENT_MODE, sLinkMode, 0, sFrame, sTransfers };
				linkSio_send(LINK_PKT_EVENT, &ev, sizeof(ev));
			}
			break;
		}
		case LINK_PKT_CHILD_CMD:
		{
			if (len == sizeof(LinkCommandPayload)) // queue space was checked in linkSio_canAccept
			{
				// The payload sits at an odd address (packet byte 3), and ARMv5 can't do unaligned
				// halfword loads, so copy it into an aligned struct first.
				LinkCommandPayload cmd;
				for (u32 i = 0; i < sizeof(cmd); i++)
				{
					((u8*)&cmd)[i] = payload[i];
				}
				u32 head = sChildQHead;
				for (int i = 0; i < 8; i++)
				{
					sChildQ[head][i] = cmd.words[i];
				}
				__asm__ volatile("" ::: "memory");
				sChildQHead = (head + 1) & (LINK_CHILDQ_SIZE - 1);
			}
			break;
		}
		default:
		{
			break; // unknown types are ignored
		}
	}
}

// Whether a packet can be handled now without dropping its output: its reply must fit in the TX
// ring, and a child command needs room in the child queue.
static bool linkSio_canAccept(u8 type, u32 len)
{
	u32 txFree = linkRing_free(linkTxRing());
	switch (type)
	{
		case LINK_PKT_PING:      return txFree >= len + LINK_PKT_OVERHEAD;
		case LINK_PKT_HELLO_REQ: return txFree >= sizeof(LinkHelloPayload) + LINK_PKT_OVERHEAD;
		case LINK_PKT_OPTIONS:   return txFree >= sizeof(LinkEventPayload) + LINK_PKT_OVERHEAD;
		case LINK_PKT_CHILD_CMD: return ((sChildQHead + 1) & (LINK_CHILDQ_SIZE - 1)) != sChildQTail;
		default:                 return true;
	}
}

// Parse every complete packet in the RX ring (produced by the ARM7 from CDC OUT).
// Back-pressure: a packet is only consumed once its reply fits in the TX ring. Otherwise it stays in
// the RX ring until a later frame; when the RX ring fills, the ARM7 stops draining CDC OUT and the
// host is NAKed. Nothing is dropped.
static void linkSio_processRx(void)
{
	LinkRing* rx = linkRxRing();
	while (true)
	{
		u32 avail = linkRing_count(rx);
		if (avail == 0)
		{
			break;
		}
		u8 b;
		if (linkRing_peek(rx, 0) != LINK_PKT_SYNC)
		{
			linkRing_pop(rx, &b);
			sRxResyncBytes++;
			continue;
		}
		if (avail < 3)
		{
			break; // header not complete yet
		}
		u32 total = linkRing_peek(rx, 2) + LINK_PKT_OVERHEAD;
		if (avail < total)
		{
			break; // rest of the packet not here yet
		}
		for (u32 i = 0; i < total; i++)
		{
			sRxPkt[i] = linkRing_peek(rx, i);
		}
		u32 len = total - LINK_PKT_OVERHEAD;
		if (linkPkt_crc8(0, &sRxPkt[1], len + 2) != sRxPkt[total - 1])
		{
			// Bad packet: drop only the sync byte and resync from the next byte, in case the
			// "packet" was really a stray A5 inside other data.
			linkRing_pop(rx, &b);
			sRxCrcErrors++;
			continue;
		}
		if (!linkSio_canAccept(sRxPkt[1], len))
		{
			break; // no room for its output yet: leave the packet queued
		}
		for (u32 i = 0; i < total; i++)
		{
			linkRing_pop(rx, &b);
		}
		sRxPackets++;
		linkSio_handlePacket(sRxPkt[1], &sRxPkt[3], len);
	}
}

static void linkSio_sendHello(void)
{
	LinkHelloPayload hello;
	const char magic[8] = { 'G', 'B', 'R', '3', 'L', 'I', 'N', 'K' };
	for (int i = 0; i < 8; i++)
	{
		hello.magic[i] = magic[i];
	}
	hello.version = LINK_PROTOCOL_VERSION;
	hello.dsiMode = (((*(vu32*)0x04004000) & 3) == 1) ? 1 : 0;
	hello.reserved[0] = 0;
	hello.reserved[1] = 0;
	hello.rxRingAddr = (u32)linkRxRing();
	hello.txRingAddr = (u32)linkTxRing();
	linkSio_send(LINK_PKT_HELLO, &hello, sizeof(hello));
}

void linkSio_init(void)
{
	linkSio_sendHello();
}

static u32 sFirstVBlankReported;

void linkSio_vblank(void)
{
	if (!sFirstVBlankReported)
	{
		sFirstVBlankReported = 1;
		*(vu32*)linkRing_accessAddr(LINK_ARM9_STAGE_ADDR) = 15; // (Thumb: no mcr; uncached alias)
		u32 stage[2] = { 15, 0 }; // boot stage 15: the game is running (main.cpp sends 1-14)
		linkSio_send(LINK_PKT_BOOT_STAGE, stage, sizeof(stage));
	}
	sFrame++;
	linkSio_processRx();
	linkSio_flushLinkOut(); // link data first: it must never be dropped
	linkSio_flushLog();

	if (sFrame % 60 == 0)
	{
		LinkArm9HeartbeatPayload hb;
		hb.frame = sFrame;
		hb.rxPackets = sRxPackets;
		hb.rxCrcErrors = sRxCrcErrors;
		hb.rxResyncBytes = sRxResyncBytes;
		hb.txDrops = sTxDrops;
		hb.logRecords = sLogRecords;
		hb.logDrops = sLogDrops;
		hb.linkMode = sLinkMode;
		hb.linkPhase = sLinkPhase;
		hb.logEnabled = sLogEnabled;
		hb.reserved = 0;
		hb.transfers = sTransfers;
		hb.bursts = sBursts;
		hb.masterCmds = sMasterCmds;
		hb.childCmds = sChildCmds;
		hb.linkDrops = sLinkDrops;
		linkSio_send(LINK_PKT_ARM9_HB, &hb, sizeof(hb));
	}
}
