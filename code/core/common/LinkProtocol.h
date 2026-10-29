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

#define LINK_PROTOCOL_VERSION 4

// Packet types (ASCII for readable hex dumps).
#define LINK_PKT_HELLO      'S' // ARM9 -> PC, at boot and on request.  LinkHelloPayload
#define LINK_PKT_HELLO_REQ  'G' // PC -> ARM9, asks for a HELLO (send this after opening the port:
                                //   the boot HELLO may have gone to an earlier session).
#define LINK_PKT_ARM9_HB    'N' // ARM9 -> PC, every 60 frames.         LinkArm9HeartbeatPayload
#define LINK_PKT_ARM7_HB    'H' // ARM7 -> PC, every 60 VBlanks.        LinkArm7HeartbeatPayload
#define LINK_PKT_PING       'P' // PC -> ARM9, any payload.
#define LINK_PKT_PONG       'Q' // ARM9 -> PC, echoes the PING payload unchanged.
#define LINK_PKT_SIO_LOG    'L' // ARM9 -> PC, n x LinkSioLogRecord (Phase 2a).
#define LINK_SIO_LOG_PER_PKT 31 // 31 * 8 = 248 bytes of payload
#define LINK_PKT_OPTIONS    'O' // PC -> ARM9, LinkOptionsPayload (link mode, logging).
#define LINK_PKT_CHILD_CMD  'C' // PC -> ARM9, LinkCommandPayload: one 8-word command for the emulated
                                //   child to send in a future burst (queued, back-pressured).
#define LINK_PKT_MASTER_CMD 'M' // ARM9 -> PC, LinkCommandPayload: a non-zero command the game (master)
                                //   sent in one burst.
#define LINK_PKT_EVENT      'E' // ARM9 -> PC, LinkEventPayload: link-layer state changes.
#define LINK_PKT_BOOT_STAGE 'B' // ARM9 -> PC, <stage:u32 arg:u32>: boot progress (diagnostics). Sent
                                //   from ARM code in main.cpp, so it works even if Thumb code doesn't.

// Link modes (LinkOptionsPayload.mode).
#define LINK_MODE_STOCK     0   // GBARunner3's original behaviour: multi mode reports "child, parent
                                //   present" and transfers never complete. Default at boot.
#define LINK_MODE_PARTNER   1   // The DS game is the master; the DS emulates the child's link layer
                                //   (handshake, checksum, absent players) and plays queued 'C' commands.

#define LINK_OPT_SIO_LOG    1   // LinkOptionsPayload.flags: SIO register log enabled (default on)

typedef struct
{
	u8 mode;              // LINK_MODE_*
	u8 flags;             // LINK_OPT_*
	u8 reserved[2];
} LinkOptionsPayload;

typedef struct
{
	u32 burst;            // ARM9 burst counter (M) / ignored (C)
	u16 words[8];         // the 8 data words of one link.c command (CMD_LENGTH)
} LinkCommandPayload;

// Link events (LinkEventPayload.code).
#define LINK_EVENT_RESET        1 // link layer reset (game wrote SIOCNT without IRQ enable / mode change)
#define LINK_EVENT_HANDSHAKE    2 // master sent MASTER_HANDSHAKE (0x8FFF): handshake completes
#define LINK_EVENT_DATA_START   3 // first data-phase burst begins
#define LINK_EVENT_MODE         4 // link mode changed (arg = new mode)

typedef struct
{
	u8  code;             // LINK_EVENT_*
	u8  arg;
	u16 reserved;
	u32 frame;            // ARM9 frame
	u32 transfers;        // transfers so far
} LinkEventPayload;

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
	u32 logRecords;       // SIO log records captured
	u32 logDrops;         // SIO log records lost (local log buffer full)
	u8  linkMode;         // LINK_MODE_*
	u8  linkPhase;        // 0 idle/handshake, 1 post-handshake, 2 data
	u8  logEnabled;
	u8  reserved;
	u32 transfers;        // emulated multiplayer transfers completed
	u32 bursts;           // data-phase bursts (9 transfers each)
	u32 masterCmds;       // non-zero master commands queued for the PC
	u32 childCmds;        // child commands played from the PC queue
	u32 linkDrops;        // M/E records lost (out queue full) - should stay 0
} LinkArm9HeartbeatPayload;

// One guest access to a link-related register (Phase 2a observation). A 32-bit access is logged
// as two records (reg, reg + 2) with LINK_SIO_LOG_FLAG_32 set.
typedef struct
{
	u16 frame;            // ARM9 frame counter (low 16 bits)
	u16 vcount;           // DS VCOUNT at the time of the access (0-262): position within the frame
	u8  reg;              // register offset - 0x100 (0x0C/0x0E TM3CNT_L/H, 0x20-0x2A SIO, 0x34 RCNT)
	u8  flags;            // LINK_SIO_LOG_FLAG_*
	u16 value;            // value written by the guest / returned to the guest
} LinkSioLogRecord;

#define LINK_SIO_LOG_FLAG_LOAD 1 // load (only logged when the value changed); else store
#define LINK_SIO_LOG_FLAG_32   2 // half of a 32-bit access

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
	u32 arm9Stage;        // ARM9 boot progress word (LINK_ARM9_STAGE_ADDR), read by the ARM7
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
