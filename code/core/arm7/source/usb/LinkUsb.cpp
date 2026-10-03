#include "common.h"
#include <libtwl/rtos/rtosIrq.h>
#include <libtwl/rtos/rtosThread.h>
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "usb_descriptors.h"
#include "LinkUsb.h"
#include "../../common/LinkRing.h"

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
// Phase 1 (loopback): bytes from the PC are echoed back (RX ring -> TX ring -> CDC IN).
// Phase 2 replaces the echo with the ARM9 SIO bridge; the CDC <-> ring shuttling stays the same.

#define TX_BATCH 64
#define HB_MSG_MAX 72 // "HB " + 5 fields of up to "x=" + 8 hex digits + space

static u8 sTxBuf[TX_BATCH];

// Diagnostics, reported by the heartbeat (all hex).
static volatile u32 sCardIrqCount = 0;    // i: card-IRQ batches handled by the DCD thread
static volatile u32 sRxCallbackCount = 0; // r: tud_cdc_rx_cb calls (OUT data received)
static volatile u32 sEchoByteCount = 0;   // e: bytes echoed PC -> DS -> PC
static volatile u32 sPumpCallCount = 0;   // p: pump invocations
static volatile u32 sVBlankCount = 0;     // v: VBlank ticks from the ARM7 main loop (60 Hz)
static u32 sLastHeartbeatVBlank = 0;
static u32 sLastRxVBlank = 0;
static volatile bool sPumpDeferred = false; // a deferred pump is queued and not yet run

// Move as many bytes as fit from the tinyusb CDC OUT FIFO into the shared RX ring. Bytes that
// don't fit stay in the tinyusb FIFO (which back-pressures the host) and are picked up by the
// next pump, so nothing is dropped.
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
		sLastRxVBlank = sVBlankCount;
	}
}

// Append "tag<value-as-hex> " to msg. Minimal: avoids newlib snprintf, whose code size
// overflows the ARM7 IWRAM.
static void linkAppendField(char* msg, int& p, const char* tag, u32 val)
{
	static const char digits[] = "0123456789abcdef";
	char buf[8];
	int n = 0;
	do
	{
		buf[n++] = digits[val & 0xF];
		val >>= 4;
	} while (val != 0 && n < 8);
	while (*tag && p < HB_MSG_MAX - 1)
		msg[p++] = *tag++;
	while (n > 0 && p < HB_MSG_MAX - 1)
		msg[p++] = buf[--n];
	if (p < HB_MSG_MAX - 1)
		msg[p++] = ' ';
}

static void linkUsbSendHeartbeat(void)
{
	char msg[HB_MSG_MAX];
	int p = 0;
	const char* pre = "HB ";
	while (*pre && p < HB_MSG_MAX - 1)
		msg[p++] = *pre++;
	linkAppendField(msg, p, "i=", sCardIrqCount);
	linkAppendField(msg, p, "r=", sRxCallbackCount);
	linkAppendField(msg, p, "e=", sEchoByteCount);
	linkAppendField(msg, p, "p=", sPumpCallCount);
	linkAppendField(msg, p, "v=", sVBlankCount);
	msg[p - 1] = '\n'; // replace the trailing space
	if (tud_cdc_write_available() >= (u32)p)
	{
		tud_cdc_write(msg, (u32)p);
		tud_cdc_write_flush();
	}
}

// Must only be called from the task thread (i.e. from a tinyusb callback or deferred call).
void linkUsbPump(void)
{
	sPumpCallCount++;
	LinkRing* rx = linkRxRing();
	LinkRing* tx = linkTxRing();
	u8 b;

	linkUsbDrainCdcRx(); // pick up anything left in the CDC FIFO while the RX ring was full

	// Phase 1 echo: PC -> RX ring -> TX ring. (Removed in Phase 2; ARM9 consumes the RX ring.)
	while (!linkRing_isFull(tx) && linkRing_pop(rx, &b))
	{
		linkRing_push(tx, b);
		++sEchoByteCount;
	}

	if (!tud_cdc_connected()) // DTR not asserted: nobody is listening
	{
		return;
	}

	// Drain the TX ring to the CDC IN endpoint (DS -> PC). Only pop what the CDC TX FIFO can
	// take: tud_cdc_write() silently drops the excess.
	u32 room = tud_cdc_write_available();
	if (room > TX_BATCH)
	{
		room = TX_BATCH;
	}
	u32 count = 0;
	while (count < room && linkRing_pop(tx, &sTxBuf[count]))
	{
		++count;
	}
	if (count > 0)
	{
		tud_cdc_write(sTxBuf, count);
		tud_cdc_write_flush();
	}

	// Heartbeat every ~1 s (60 VBlanks); muted for 2 s after received data so it doesn't get
	// mixed into a loopback echo.
	if (sVBlankCount - sLastHeartbeatVBlank >= 60 && sVBlankCount - sLastRxVBlank >= 120)
	{
		sLastHeartbeatVBlank = sVBlankCount;
		linkUsbSendHeartbeat();
	}
}

// tinyusb CDC callbacks. NB: this tinyusb version's signature is tud_cdc_rx_cb(uint8_t itf).
// They are declared extern "C" in cdc_device.h, so a definition with a different parameter list
// would silently become an unrelated C++ overload that tinyusb never calls.
void tud_cdc_rx_cb(uint8_t itf)
{
	(void)itf;
	sRxCallbackCount++;
	linkUsbPump();
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
