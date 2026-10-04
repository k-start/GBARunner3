#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Link-cable bridge, ARM9 side. Phase 2a-0: packet transport only (no SIO emulation yet).

// Send the HELLO packet. Call once at boot, before the VM starts (IRQs off).
void linkSio_init(void);

// Once per DS frame, from the VBlank IRQ (Emulator/VBlankIrq.s) on the DTCM IRQ stack
// (only 288 bytes: keep this shallow, buffers are static). Processes packets from the PC and sends
// the ARM9 heartbeat every 60 frames.
void linkSio_vblank(void);

#ifdef __cplusplus
}
#endif
