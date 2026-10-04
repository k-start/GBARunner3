#pragma once
#include "LinkRing.h"

// Packet framing for the USB CDC link (both directions), shared by ARM7, ARM9 and (documented
// for) the PC tools in tools/link_mon.py. Keep the two in sync.
//
//   A5 <type:u8> <len:u8> <payload[len]> <crc8>
//
// crc8: polynomial 0x07, init 0x00, over <type> <len> <payload>. All multi-byte payload fields are
// little-endian. A receiver resynchronises by discarding bytes until it sees A5 followed by a
// packet with a valid CRC.
//
// Writers publish a whole packet to a ring at once (linkRing_writeBlock), so a reader never sees a
// partial packet. The ARM7 forwards the TX ring to USB in whole packets only, so it can interleave
// its own packets (heartbeat) between them without corrupting the stream.

#define LINK_PKT_SYNC        0xA5
#define LINK_PKT_MAX_PAYLOAD 255
#define LINK_PKT_OVERHEAD    4 // sync + type + len + crc
#define LINK_PKT_MAX_SIZE    (LINK_PKT_MAX_PAYLOAD + LINK_PKT_OVERHEAD)

#define LINK_PROTOCOL_VERSION 2

// Packet types (ASCII for readable hex dumps).
#define LINK_PKT_HELLO      'S' // ARM9 -> PC, at boot and on request.  LinkHelloPayload
#define LINK_PKT_HELLO_REQ  'G' // PC -> ARM9, asks for a HELLO (send this after opening the port:
                                //   the boot HELLO may have gone to an earlier session).
#define LINK_PKT_ARM9_HB    'N' // ARM9 -> PC, every 60 frames.         LinkArm9HeartbeatPayload
#define LINK_PKT_ARM7_HB    'H' // ARM7 -> PC, every 60 VBlanks.        LinkArm7HeartbeatPayload
#define LINK_PKT_PING       'P' // PC -> ARM9, any payload.
#define LINK_PKT_PONG       'Q' // ARM9 -> PC, echoes the PING payload unchanged.
#define LINK_PKT_SIO_LOG    'L' // ARM9 -> PC, SIO register log records (Phase 2a).

typedef struct
{
	char magic[8];        // "GBR3LINK"
	u8   version;         // LINK_PROTOCOL_VERSION
	u8   dsiMode;         // 1 if the ARM9 detected TWL mode
	u8   reserved[2];
	u32  rxRingAddr;      // address the ARM9 uses for the RX ring (its uncached alias)
	u32  txRingAddr;      // address the ARM9 uses for the TX ring (its uncached alias)
} LinkHelloPayload;

typedef struct
{
	u32 frame;            // ARM9 VBlank count
	u32 rxPackets;        // valid packets received from the PC
	u32 rxCrcErrors;      // packets dropped for a bad CRC
	u32 rxResyncBytes;    // bytes discarded while hunting for LINK_PKT_SYNC
	u32 txDrops;          // packets the ARM9 could not queue (TX ring full)
} LinkArm9HeartbeatPayload;

typedef struct
{
	u32 vblank;           // ARM7 VBlank count
	u32 cardIrqs;         // card-IRQ batches handled by the DCD thread
	u32 usbRxBytes;       // bytes received from the PC (CDC OUT)
	u32 txPackets;        // TX-ring packets forwarded to the PC
	u32 txResyncBytes;    // TX-ring bytes discarded because they weren't at a packet start
	u16 rxRingUsed;       // bytes waiting in the RX ring for the ARM9
	u16 txRingUsed;       // bytes waiting in the TX ring for the ARM7
	u32 pumps;            // pump invocations
	u32 dtrConnects;      // DTR rising edges (host opened the port); TX FIFO is cleared on each
} LinkArm7HeartbeatPayload;

static inline u8 linkPkt_crc8(u8 crc, const u8* data, u32 len)
{
	for (u32 i = 0; i < len; i++)
	{
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++)
			crc = (crc & 0x80) ? (u8)((crc << 1) ^ 0x07) : (u8)(crc << 1);
	}
	return crc;
}

// Build a framed packet into out (at least len + LINK_PKT_OVERHEAD bytes). Returns its size.
static inline u32 linkPkt_build(u8* out, u8 type, const void* payload, u32 len)
{
	const u8* p = (const u8*)payload;
	out[0] = LINK_PKT_SYNC;
	out[1] = type;
	out[2] = (u8)len;
	for (u32 i = 0; i < len; i++)
		out[3 + i] = p[i];
	out[3 + len] = linkPkt_crc8(0, &out[1], len + 2);
	return len + LINK_PKT_OVERHEAD;
}
