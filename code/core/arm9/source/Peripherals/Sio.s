.section ".itcm", "ax"
.altmacro

#include "AsmMacros.inc"
#include "GbaIoRegOffsets.h"

arm_func emu_regSioCntSioMultiSendStore32
    ldr r11,= (emu_ioRegisters + GBA_REG_OFFS_SIOMULTI_SEND)
    mov r12, r9, ror #16
    strh r12, [r11]
    mov r9, r12, lsr #16

arm_func emu_regSioCntStore16
    ldr r11,= (emu_ioRegisters + GBA_REG_OFFS_SIOCNT)
    ldrh r10, [r11, #(GBA_REG_OFFS_RCNT - GBA_REG_OFFS_SIOCNT)]
    and r12, r9, #0x3000
    and r10, r10, #0xC000
    orr r10, r10, r12
    mov r10, r10, lsr #12
    cmp r10, #8
        andlo r10, r10, #3
        andhs r10, r10, #0xC
    cmp r10, #2
        bhs regSioCntStore16_multi

    // normal mode
    orr r9, r9, #(1 << 2) // SI
    and r12, r9, #0x81
    cmp r12, #0x81
        bne regSioCntStore16_end
    bic r9, r9, #0x80
    strh r9, [r11]
    tst r9, #0x4000 // irq
        bxeq lr
    ldr r12,= vm_emulatedIfImeIe
    ldr r10, [r12]
    orr r10, r10, #(1 << 7) // SIO IRQ
    str r10, [r12]
    b emu_updateIrqs

regSioCntStore16_multi:
        bne regSioCntStore16_end // UART / JOY bus: store as-is
    // Multiplayer mode (link cable): the status bits and, with the link partner enabled, the whole
    // transfer are handled by linkSio_multiCntStore (LinkSio.c). It returns the new SIOCNT value,
    // with bit 31 set if the SIO IRQ must be raised. r11 (&SIOCNT) is callee-saved.
    push {r0-r3,lr}
    mov r0, r9
    bl linkSio_multiCntStore
    mov r9, r0
    pop {r0-r3,lr}
    strh r9, [r11]
    tst r9, #(1 << 31)
        bxeq lr
    ldr r12,= vm_emulatedIfImeIe
    ldr r10, [r12]
    orr r10, r10, #(1 << 7) // SIO IRQ
    str r10, [r12]
    b emu_updateIrqs

regSioCntStore16_end:
    strh r9, [r11]
    bx lr

// ---------------------------------------------------------------------------------------------
// Link cable, Phase 2a: observation hooks. Each wrapper logs the guest access (LinkSio.c) and
// then behaves exactly like the original handler, so emulation is unchanged.
// Convention (store/load handlers): r8 = address, r9 = value, r10-r12 scratch, return via lr.
// C calls follow the existing pattern (e.g. emu_dmaCntHStore16): save r0-r3,lr; r4-r11 are
// callee-saved; r12 is scratch.
// ---------------------------------------------------------------------------------------------

.macro link_log func, width
    push {r0-r3,lr}
    mov r0, r8
    mov r1, r9
    mov r2, #\width
    bl \func
    pop {r0-r3,lr}
.endm

// stores (value logged as written by the guest, before any masking)
arm_func emu_linkSioCntStore16
    link_log linkSio_logStore, 2
    b emu_regSioCntStore16

arm_func emu_linkSioCntSioMultiSendStore32
    link_log linkSio_logStore, 4
    b emu_regSioCntSioMultiSendStore32

arm_func emu_linkIoStore16
    link_log linkSio_logStore, 2
    b emu_ioStore16

arm_func emu_linkIoStore32
    link_log linkSio_logStore, 4
    b emu_ioStore32

arm_func emu_linkTimerCntLStore16
    link_log linkSio_logStore, 2
    b emu_timerCntLStore16

arm_func emu_linkTimerCntHStore16
    link_log linkSio_logStore, 2
    b emu_timerCntHStore16

arm_func emu_linkTimerCntStore32
    link_log linkSio_logStore, 4
    b emu_timerCntStore32

// loads (value logged as returned to the guest; LinkSio.c only records changes)
arm_func emu_linkIoLoad16
    ldr r11,= (emu_ioRegisters - 0x04000000)
    ldrh r9, [r11, r8]
    link_log linkSio_logLoad, 2
    bx lr

arm_func emu_linkIoLoad32
    ldr r11,= (emu_ioRegisters - 0x04000000)
    ldr r9, [r11, r8]
    link_log linkSio_logLoad, 4
    bx lr
