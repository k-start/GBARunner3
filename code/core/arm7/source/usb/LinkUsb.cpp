#include "common.h"
#include <libtwl/rtos/rtosIrq.h>
#include <libtwl/rtos/rtosThread.h>
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "usb_descriptors.h"
#include "LinkUsb.h"
#include "../../common/LinkProtocol.h"

// DSpico link-cable USB bridge (ARM7).
//
// Threading model — deliberately IDENTICAL to dspico-usb-examples/mass-storage:
//   1. The DCD thread (dcd_dspico.cpp, priority 17) blocks on card IRQs and calls tinyusb's
//      dcd_event_* entry points, which enqueue events into the usbd queue.
//   2. The task thread (this file, priority 18) runs `while (true) tud_task();`. tud_task()
//      blocks on the queue's event, so it uses no CPU when idle.
// libtwl: a HIGHER number is a HIGHER priority. The task thread must be ABOVE the DCD thread (as
// in the examples: 3 vs 2): every enqueue then immediately preempts the DCD thread and the event
// is handled before the DCD thread reads the next one from the DSpico. tinyusb (stale-SETUP
// skipping) and DSPicoUsbIn/OutEndpoint are written for that ordering. (Note: the DSpico
// delivers no SOF events, so the DCD thread only wakes on real USB traffic.)
// (Other priorities: idle 0, FS IPC 6, ARM7 main loop 16, GBA sound mixer 20.)
//
// ALL link work runs inside the task thread, so tinyusb is only ever called from one thread:
//   - tud_cdc_rx_cb          (OUT data arrived)      -> pump
//   - tud_cdc_tx_complete_cb (IN transfer finished)  -> pump
//   - once per VBlank, the ARM7 main loop defers a pump call into the task thread via
//     usbd_defer_func (so TX data produced by the ARM9 and the heartbeat flow without USB events).
//
// Data path (packet framing: common/LinkProtocol.h):
//   PC -> CDC OUT -> RX ring -> ARM9 (parses packets)
//   ARM9 -> TX ring -> (whole packets) -> CDC IN -> PC, plus the ARM7's own heartbeat packets.

// Diagnostics, reported in the ARM7 heartbeat packet (LinkArm7HeartbeatPayload).
static volatile u32 sCardIrqCount = 0;    // card-IRQ batches handled by the DCD thread
static volatile u32 sUsbRxBytes = 0;      // bytes received from the PC (CDC OUT)
static volatile u32 sTxPackets = 0;       // TX-ring packets forwarded to the PC
static volatile u32 sTxResyncBytes = 0;   // TX-ring bytes discarded (not at a packet start)
static volatile u32 sPumpCallCount = 0;   // pump invocations
static volatile u32 sVBlankCount = 0;     // VBlank ticks from the ARM7 main loop (60 Hz)
static volatile u32 sDtrConnects = 0;     // DTR rising edges
static u32 sLastHeartbeatVBlank = 0;
static volatile bool sPumpDeferred = false; // a deferred pump is queued and not yet run

static u8 sPktBuf[LINK_PKT_MAX_SIZE];

// Move as many bytes as fit from the tinyusb CDC OUT FIFO into the shared RX ring (consumed by the
// ARM9). Bytes that don't fit stay in the tinyusb FIFO, which back-pressures the host, and are
// picked up by the next pump, so nothing is dropped.
static void linkUsbDrainCdcRx(void)
{
	LinkRing* rx = linkRxRing();
	while (!linkRing_isFull(rx) && tud_cdc_available() > 0)
	{
		u8 b;
		if (tud_cdc_read(&b, 1) != 1)
		{
			break;
		}
		linkRing_push(rx, b);
		sUsbRxBytes++;
	}
}

// Forward complete packets from the TX ring (produced by the ARM9) to the CDC IN endpoint. Only
// whole packets are forwarded, so packets the ARM7 writes itself can go in between them.
// Returns true if anything was written.
static bool linkUsbForwardTxPackets(void)
{
	LinkRing* tx = linkTxRing();
	bool wrote = false;
	while (true)
	{
		u32 avail = linkRing_count(tx);
		if (avail == 0)
		{
			break;
		}
		if (linkRing_peek(tx, 0) != LINK_PKT_SYNC)
		{
			u8 junk;
			linkRing_pop(tx, &junk); // not a packet start: discard and resync
			sTxResyncBytes++;
			continue;
		}
		if (avail < 3)
		{
			break; // can't happen with linkRing_writeBlock producers, but be safe
		}
		u32 total = linkRing_peek(tx, 2) + LINK_PKT_OVERHEAD;
		if (avail < total || tud_cdc_write_available() < total)
		{
			break; // wait for the rest / for room in the CDC FIFO
		}
		for (u32 i = 0; i < total; i++)
		{
			linkRing_pop(tx, &sPktBuf[i]);
		}
		tud_cdc_write(sPktBuf, total);
		sTxPackets++;
		wrote = true;
	}
	return wrote;
}

static bool linkUsbSendHeartbeat(void)
{
	LinkArm7HeartbeatPayload hb;
	hb.vblank = sVBlankCount;
	hb.cardIrqs = sCardIrqCount;
	hb.usbRxBytes = sUsbRxBytes;
	hb.txPackets = sTxPackets;
	hb.txResyncBytes = sTxResyncBytes;
	hb.rxRingUsed = (u16)linkRing_count(linkRxRing());
	hb.txRingUsed = (u16)linkRing_count(linkTxRing());
	hb.pumps = sPumpCallCount;
	hb.dtrConnects = sDtrConnects;
	u32 size = linkPkt_build(sPktBuf, LINK_PKT_ARM7_HB, &hb, sizeof(hb));
	if (tud_cdc_write_available() < size)
	{
		return false;
	}
	tud_cdc_write(sPktBuf, size);
	return true;
}

// Must only be called from the task thread (i.e. from a tinyusb callback or deferred call).
void linkUsbPump(void)
{
	sPumpCallCount++;

	linkUsbDrainCdcRx(); // PC -> RX ring (anything left in the CDC FIFO while the ring was full)

	if (!tud_cdc_connected()) // DTR not asserted: nobody is listening; leave TX data queued
	{
		return;
	}

	bool wrote = linkUsbForwardTxPackets(); // ARM9 -> PC

	// ARM7 heartbeat every ~1 s (60 VBlanks), between packets.
	if (sVBlankCount - sLastHeartbeatVBlank >= 60)
	{
		sLastHeartbeatVBlank = sVBlankCount;
		wrote |= linkUsbSendHeartbeat();
	}

	if (wrote)
	{
		tud_cdc_write_flush();
	}
}

// tinyusb CDC callbacks. NB: this tinyusb version's signature is tud_cdc_rx_cb(uint8_t itf).
// They are declared extern "C" in cdc_device.h, so a definition with a different parameter list
// would silently become an unrelated C++ overload that tinyusb never calls.
void tud_cdc_rx_cb(uint8_t itf)
{
	(void)itf;
	linkUsbPump();
}

// Host opened (DTR high) or closed (DTR low) the port. tinyusb makes the CDC TX FIFO overwritable
// while DTR is low, and anything queued for a previous session may end mid-packet, so start every
// new session with an empty FIFO. Whole packets still waiting in the TX ring are kept.
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	(void)itf;
	(void)rts;
	if (dtr)
	{
		sDtrConnects++;
		tud_cdc_write_clear();
		linkUsbPump();
	}
}

void tud_cdc_tx_complete_cb(uint8_t itf)
{
	(void)itf;
	linkUsbPump();
}

static void linkUsbDeferredPump(void* param)
{
	(void)param;
	sPumpDeferred = false;
	linkUsbPump();
}

static rtos_thread_t sTusbTaskThread;
static u32 sTusbTaskThreadStack[512];

static void tusbTaskThreadMain(void* arg)
{
	(void)arg;
	while (true)
	{
		tud_task();
	}
}

// Called by the DCD thread after handling a card-IRQ batch. Diagnostics only.
void linkUsbNotifyActivity(void)
{
	sCardIrqCount++;
}

// Called by the ARM7 main loop on every VBlank.
void linkUsbVBlankTick(void)
{
	sVBlankCount++;
	if (!tud_inited())
	{
		return;
	}
	// The usbd queue's producers are otherwise only the DCD thread; IRQs off so it can't preempt
	// us halfway through the enqueue. At most one deferred pump is queued at a time.
	u32 irq = rtos_disableIrqs();
	if (!sPumpDeferred)
	{
		sPumpDeferred = true;
		usbd_defer_func(linkUsbDeferredPump, NULL, false);
	}
	rtos_restoreIrqs(irq);
}

void linkUsbInit(void)
{
	// The shared rings live in fixed EWRAM that is not zeroed on a cold boot.
	linkRing_reset(linkRxRing());
	linkRing_reset(linkTxRing());

	tusb_rhport_init_t devInit =
	{
		.role = TUSB_ROLE_DEVICE,
		.speed = TUSB_SPEED_AUTO
	};
	tusb_init(0, &devInit); // starts the DCD USB thread (dcd_dspico.cpp, priority 17).

	// Priority 18: above the DCD thread (see the threading notes at the top of this file).
	rtos_createThread(&sTusbTaskThread, 18, tusbTaskThreadMain, NULL,
	                  sTusbTaskThreadStack, sizeof(sTusbTaskThreadStack));
	rtos_wakeupThread(&sTusbTaskThread);
}
