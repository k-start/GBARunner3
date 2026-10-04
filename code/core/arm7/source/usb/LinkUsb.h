#pragma once

// Initialise the tinyusb stack (device mode, CDC). This starts the DCD USB thread
// (dcd_dspico.cpp) and the tinyusb task thread (LinkUsb.cpp).
void linkUsbInit(void);

// Pump the link: CDC OUT -> RX ring, whole TX-ring packets -> CDC IN, plus the ARM7 heartbeat.
// Must only be called from the tinyusb task thread (CDC callbacks / deferred calls).
void linkUsbPump(void);

// Diagnostics: count a handled card-IRQ batch. Called by the DCD thread (dcd_dspico.cpp).
void linkUsbNotifyActivity(void);

// Defer one pump call into the tinyusb task thread, so ring traffic and the heartbeat flow even
// without USB events. Called by the ARM7 main loop on VBlank.
void linkUsbVBlankTick(void);
