#include "common.h"
#include "LinkProtocol.h"
#include "LinkSio.h"

// TX-ring producers on the ARM9: linkSio_init (before IRQs are enabled) and linkSio_vblank (VBlank
// IRQ). They can't run concurrently. When Phase 2a adds producers outside the IRQ (SIO register
// store handlers), those must disable IRQs around linkSio_send.

static u32 sFrame;
static u32 sRxPackets;
static u32 sRxCrcErrors;
static u32 sRxResyncBytes;
static u32 sTxDrops;

static u8 sRxPkt[LINK_PKT_MAX_SIZE];
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
		default:
		{
			break; // unknown types are ignored
		}
	}
}

// Parse every complete packet in the RX ring (produced by the ARM7 from CDC OUT).
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

void linkSio_vblank(void)
{
	sFrame++;
	linkSio_processRx();

	if (sFrame % 60 == 0)
	{
		LinkArm9HeartbeatPayload hb;
		hb.frame = sFrame;
		hb.rxPackets = sRxPackets;
		hb.rxCrcErrors = sRxCrcErrors;
		hb.rxResyncBytes = sRxResyncBytes;
		hb.txDrops = sTxDrops;
		linkSio_send(LINK_PKT_ARM9_HB, &hb, sizeof(hb));
	}
}
