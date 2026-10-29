#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Link-cable bridge, ARM9 side: packet transport, SIO observation log (2a), emulated link partner (2b).

// Send the HELLO packet. Call once at boot, before the VM starts (IRQs off).
void linkSio_init(void);

// Once per DS frame, from the VBlank IRQ (Emulator/VBlankIrq.s) on the DTCM IRQ stack
// (only 288 bytes: keep this shallow, buffers are static). Processes packets from the PC and sends
// the ARM9 heartbeat every 60 frames.
void linkSio_vblank(void);

// SIO observation (Phase 2a), called from the guest IO handlers in Peripherals/Sio.s.
// addr = GBA IO address, value = value written / returned, width = 2 or 4 bytes.
void linkSio_logStore(u32 addr, u32 value, u32 width);
void linkSio_logLoad(u32 addr, u32 value, u32 width);

// Phase 2b: guest SIOCNT store in multiplayer mode (Peripherals/Sio.s). Returns the value to store,
// bit 31 set = raise the SIO IRQ. In LINK_MODE_PARTNER, a START completes the transfer at once, with
// the DS emulating the child's link layer.
u32 linkSio_multiCntStore(u32 value);

#ifdef __cplusplus
}
#endif
