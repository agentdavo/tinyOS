/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/* cpu_rv64.S — RISC-V M-mode boot + trap handler + context switch for miniOS.
 *
 * Entry state from QEMU -bios none -kernel: M-mode, a0 = hartid, a1 = DTB.
 * All harts enter _start. Hart 0 zeros BSS and runs .init_array, then calls
 * kernel_main_rv64(hartid). Secondaries spin on g_secondary_entry[hartid];
 * when non-zero they jump into it with a0=hartid.
 *
 * Trap vector:
 *   - Always save full interrupted GPRs (x1..x31 minus x0) + mepc/mstatus
 *     onto a per-hart IRQ stack (mscratch = top-of-irq-stack).  256-byte
 *     frame, 16-byte aligned.
 *   - If g_current_tcb[hartid] is non-null, copy the save into the TCB.
 *   - Call hal_trap_dispatch_rv64(hartid, mcause).
 *   - On return, reload g_current_tcb[hartid] (may have changed — this is
 *     how preemptive context switch happens) and restore from there.  If
 *     null, restore from the local IRQ-stack frame.
 *   - mret.
 *
 * Context switch (voluntary):
 *   cpu_context_switch_rv64(old_tcb, new_tcb) — saves callee-saved + ra/sp
 *   into old_tcb, loads from new_tcb, returns into new_tcb's saved ra.
 *   Used from cooperative yield paths.
 *
 * TCB layout (byte offsets, must match kernel::core::TCB in core.hpp):
 *   0   ..  30*8    regs[31]  (regs[i] holds x(i+1); x0 has no slot)
 *   31*8 = 248      sp        (u64, also mirrored in regs[1])
 *   32*8 = 256      pc        (u64, = mepc on entry)
 *   33*8 = 264      pstate    (u64 — rv64 stores mstatus here)
 *   264+8 and beyond: C-side fields (entry_point, arg_ptr, stack_base, ...)
 *
 * CSR bit layout reminders:
 *   mstatus.MIE  = bit 3   mstatus.MPIE = bit 7
 *   mie.MTIE     = bit 7   mie.MEIE     = bit 11
 *   mcause.INT   = bit 63  code low bits: 7 = MTI, 11 = MEI, 3 = MSI
 */

    .equ STACK_SIZE,      0x4000      /* 16 KiB per-hart boot stack */
    .equ IRQ_STACK_SIZE,  0x1000      /* 4 KiB per-hart IRQ stack */
    .equ MAX_CORES,       4
    .equ UART_THR,        0x10000000
    .equ MSTATUS_FS,      0x00006000  /* FS[1:0] = 11 (Dirty) */

    /* TCB offsets — must match kernel::core::TCB. The pstate field at
     * byte offset 264 stores mstatus on rv64 (same slot arm64 uses for
     * SPSR_EL1). */
    .equ TCB_REGS_OFF,    0
    .equ TCB_SP_OFF,      (31*8)
    .equ TCB_PC_OFF,      (31*8 + 8)
    .equ TCB_MSTATUS_OFF, (31*8 + 16)

    /* IRQ-stack frame layout (256 bytes, 16-byte aligned). Offsets here
     * index into the frame; r<N> means "slot for x<N>".  Matches the order
     * TCB regs[] uses (regs[i] = x(i+1)) so copy-to-TCB is a bulk memcpy. */
    .equ FR_X1,   0           /* ra   */
    .equ FR_X2,   8           /* sp   (saved from mscratch) */
    .equ FR_X3,  16           /* gp   */
    .equ FR_X4,  24           /* tp   */
    .equ FR_X5,  32           /* t0   */
    .equ FR_X6,  40           /* t1   */
    .equ FR_X7,  48           /* t2   */
    .equ FR_X8,  56           /* s0/fp */
    .equ FR_X9,  64           /* s1   */
    .equ FR_X10, 72           /* a0   */
    .equ FR_X11, 80           /* a1   */
    .equ FR_X12, 88
    .equ FR_X13, 96
    .equ FR_X14,104
    .equ FR_X15,112
    .equ FR_X16,120
    .equ FR_X17,128
    .equ FR_X18,136
    .equ FR_X19,144
    .equ FR_X20,152
    .equ FR_X21,160
    .equ FR_X22,168
    .equ FR_X23,176
    .equ FR_X24,184
    .equ FR_X25,192
    .equ FR_X26,200
    .equ FR_X27,208
    .equ FR_X28,216
    .equ FR_X29,224
    .equ FR_X30,232
    .equ FR_X31,240
    .equ FR_MEPC,   248
    /* 256 = FR_MEPC + 8.  We also keep mstatus/mcause in scratch regs —
     * mcause is only needed for dispatch, mstatus is restored via TCB or
     * the local slot.  To keep 16-byte alignment we round FR size up to
     * 256 even though we only truly need 252.  Using 256 simplifies the
     * addi sp,-256 prologue. */

    /* -----------------------------------------------------------------
     * Boot entry — every hart lands here (QEMU -kernel).
     * ----------------------------------------------------------------- */
    .section .text.boot
    .global _start
_start:
    /* Install trap vector ASAP so faults become visible. */
    lla     t0, trap_entry
    csrw    mtvec, t0

    /* Enable FPU (FS=Dirty) so lp64d C++ doesn't trap on F/D insns. */
    li      t0, MSTATUS_FS
    csrs    mstatus, t0

    /* Mask all interrupts until kernel is ready. */
    csrw    mie, zero
    li      t0, 0x8             /* MIE bit */
    csrc    mstatus, t0

    /* Per-hart boot stack = _stacks_start + (hartid+1)*STACK_SIZE, 16B aligned. */
    lla     t0, _stacks_start
    li      t1, STACK_SIZE
    addi    t2, a0, 1
    mul     t1, t1, t2
    add     sp, t0, t1
    andi    sp, sp, -16

    /* Fence-off: secondaries wait on g_secondary_entry[hartid] spin-table. */
    bnez    a0, .L_secondary_spin

    /* Hart 0 only: QEMU passes the DTB pointer in a1. Stash it in a global
     * BEFORE we touch BSS — not because BSS wipes a1 (it doesn't), but
     * because the C++ ctors run next and one of them could clobber a1.
     * We stash to a dedicated .data symbol so it survives the .bss zero
     * pass (which only zeroes .bss between _bss_start/_bss_end). */
    lla     t0, g_dtb_ptr
    sd      a1, 0(t0)

    /* Zero BSS on hart 0. _bss_start/_bss_end are 8-byte aligned. */
    lla     t0, _bss_start
    lla     t1, _bss_end
.L_bss_zero_loop:
    bgeu    t0, t1, .L_bss_zero_done
    sd      zero, (t0)
    addi    t0, t0, 8
    j       .L_bss_zero_loop
.L_bss_zero_done:

    /* Run C++ global ctors (.init_array). */
    lla     s0, __init_array_start
    lla     s1, __init_array_end
.L_init_loop:
    bgeu    s0, s1, .L_init_done
    ld      t0, 0(s0)
    addi    s0, s0, 8
    beqz    t0, .L_init_loop
    jalr    t0
    j       .L_init_loop
.L_init_done:

    /* kernel_main_rv64(hartid). */
    csrr    a0, mhartid
    call    kernel_main_rv64

.L_primary_park:
    wfi
    j       .L_primary_park

    /* -----------------------------------------------------------------
     * Secondary spin-table. Each secondary hart polls its slot in
     * g_secondary_entry[] (extern uint64_t[MAX_CORES], see C side).
     * When hart 0 writes a non-zero function pointer, the secondary
     * jumps to it with a0 = hartid.
     * ----------------------------------------------------------------- */
.L_secondary_spin:
    /* t3 = &g_secondary_entry[hartid] */
    lla     t3, g_secondary_entry
    slli    t4, a0, 3
    add     t3, t3, t4
1:
    /* Acquire fence so we observe hart 0's store to g_secondary_entry[]. */
    fence   r,r
    ld      t5, 0(t3)
    bnez    t5, 3f
    /* Busy-poll instead of wfi: some QEMU versions don't cleanly wake
     * secondary harts via MSIP when the M-mode boot ran without SBI,
     * leaving them parked in wfi forever.  A short nop-pause is enough
     * to not peg the host CPU while we wait for hart 0 to publish the
     * entry pointer. */
    .rept 16
    nop
    .endr
    j       1b
3:
    /* Jump with hartid in a0.  hart_main() emits the "released from spin-table"
     * phase line as its first output so we can trace per-hart bring-up from
     * the C side without smearing through raw UART writes here. */
    jr      t5

    /* -----------------------------------------------------------------
     * Trap vector.  Saves full GPR set + mepc/mstatus onto the IRQ stack;
     * optionally copies the save into g_current_tcb[hartid] if non-null;
     * dispatches to C; reloads current TCB and restores.  This makes
     * "scheduler updated g_current_tcb[hartid] to a new TCB" a transparent
     * preemptive switch.
     * ----------------------------------------------------------------- */
    .section .text
    .balign 4
    .global trap_entry
trap_entry:
    /* Swap to per-hart IRQ stack. mscratch = irq_sp_top coming in,
     * mscratch = old (interrupted) sp on exit of this instruction. */
    csrrw   sp, mscratch, sp
    addi    sp, sp, -256

    /* Save x1,x3..x31 first (x2=sp is held in mscratch; we'll recover it
     * below).  Order intentional: spill ra (x1) first so we can then clobber
     * it as a scratch. */
    sd      x1,  FR_X1(sp)
    sd      x3,  FR_X3(sp)
    sd      x4,  FR_X4(sp)
    sd      x5,  FR_X5(sp)
    sd      x6,  FR_X6(sp)
    sd      x7,  FR_X7(sp)
    sd      x8,  FR_X8(sp)
    sd      x9,  FR_X9(sp)
    sd      x10, FR_X10(sp)
    sd      x11, FR_X11(sp)
    sd      x12, FR_X12(sp)
    sd      x13, FR_X13(sp)
    sd      x14, FR_X14(sp)
    sd      x15, FR_X15(sp)
    sd      x16, FR_X16(sp)
    sd      x17, FR_X17(sp)
    sd      x18, FR_X18(sp)
    sd      x19, FR_X19(sp)
    sd      x20, FR_X20(sp)
    sd      x21, FR_X21(sp)
    sd      x22, FR_X22(sp)
    sd      x23, FR_X23(sp)
    sd      x24, FR_X24(sp)
    sd      x25, FR_X25(sp)
    sd      x26, FR_X26(sp)
    sd      x27, FR_X27(sp)
    sd      x28, FR_X28(sp)
    sd      x29, FR_X29(sp)
    sd      x30, FR_X30(sp)
    sd      x31, FR_X31(sp)

    /* Recover interrupted sp from mscratch and store it in the frame.
     * Re-prime mscratch with the irq-stack-top so re-entry is safe. */
    csrr    t0, mscratch
    sd      t0, FR_X2(sp)                   /* interrupted sp */
    /* Reset mscratch to irq_sp_top = sp + 256 (our frame top). */
    addi    t1, sp, 256
    csrw    mscratch, t1

    /* Save mepc. */
    csrr    t0, mepc
    sd      t0, FR_MEPC(sp)

    /* ---- If g_current_tcb[hartid] != 0, copy frame into TCB ---- */
    csrr    a0, mhartid
    lla     t1, g_current_tcb
    slli    t2, a0, 3
    add     t1, t1, t2
    ld      t1, 0(t1)                       /* t1 = current TCB* (or 0) */
    beqz    t1, .L_trap_skip_tcb_save

    /* memcpy 31*8=248 bytes of regs + sp-slot from frame to TCB.
     * Frame regs[] = offsets 0..240 by 8, matches TCB_REGS_OFF..+30*8. */
    addi    t2, sp, 0                       /* t2 = src (frame) */
    addi    t3, t1, TCB_REGS_OFF            /* t3 = dst (TCB regs) */
    li      t4, 31                          /* 31 dwords */
.L_trap_copy_regs:
    ld      t5, 0(t2)
    sd      t5, 0(t3)
    addi    t2, t2, 8
    addi    t3, t3, 8
    addi    t4, t4, -1
    bnez    t4, .L_trap_copy_regs

    /* Also write TCB_SP_OFF (= regs[1] content, the saved interrupted sp). */
    ld      t5, FR_X2(sp)
    sd      t5, TCB_SP_OFF(t1)

    /* mepc -> TCB.pc */
    ld      t5, FR_MEPC(sp)
    sd      t5, TCB_PC_OFF(t1)

    /* mstatus -> TCB.mstatus */
    csrr    t5, mstatus
    sd      t5, TCB_MSTATUS_OFF(t1)

.L_trap_skip_tcb_save:
    /* ---- Dispatch C handler ---- */
    csrr    a0, mhartid
    csrr    a1, mcause
    call    hal_trap_dispatch_rv64

    /* ---- Restore ---- */
    /* Reload current_tcb — may have been swapped by scheduler. */
    csrr    a0, mhartid
    lla     t1, g_current_tcb
    slli    t2, a0, 3
    add     t1, t1, t2
    ld      t1, 0(t1)
    beqz    t1, .L_trap_restore_frame

    /* Copy TCB regs[] into frame, mepc & mstatus into CSRs directly. */
    addi    t2, t1, TCB_REGS_OFF
    addi    t3, sp, 0
    li      t4, 31
.L_trap_copy_back:
    ld      t5, 0(t2)
    sd      t5, 0(t3)
    addi    t2, t2, 8
    addi    t3, t3, 8
    addi    t4, t4, -1
    bnez    t4, .L_trap_copy_back

    ld      t5, TCB_PC_OFF(t1)
    csrw    mepc, t5
    ld      t5, TCB_MSTATUS_OFF(t1)
    csrw    mstatus, t5
    j       .L_trap_restore_frame_after_csrs

.L_trap_restore_frame:
    /* No TCB — restore mepc/mstatus from the local frame. */
    ld      t0, FR_MEPC(sp)
    csrw    mepc, t0
    /* (mstatus isn't saved in the local frame; a fresh trap inherits from
     *  the CPU state we leave.  The C handler hasn't modified it.) */

.L_trap_restore_frame_after_csrs:
    /* Restore all GPRs from the frame.  We stage the thread sp into
     * mscratch FIRST using x5/t0 as the scratch, THEN restore t0 last so
     * it ends up with the interrupted value.  Leaving t0 till the end
     * avoids the classic "clobbered just-restored t0" bug.
     *
     * Also leave sp alone until after the mscratch stage — we need sp as
     * the frame base until the end. */

    /* Stage thread sp into mscratch via t0 (not yet restored). */
    ld      t0, FR_X2(sp)
    csrw    mscratch, t0                    /* mscratch = thread sp */

    /* Restore everything except t0 and sp.  Order is flexible. */
    ld      x1,  FR_X1(sp)
    ld      x3,  FR_X3(sp)
    ld      x4,  FR_X4(sp)
    ld      x6,  FR_X6(sp)
    ld      x7,  FR_X7(sp)
    ld      x8,  FR_X8(sp)
    ld      x9,  FR_X9(sp)
    ld      x10, FR_X10(sp)
    ld      x11, FR_X11(sp)
    ld      x12, FR_X12(sp)
    ld      x13, FR_X13(sp)
    ld      x14, FR_X14(sp)
    ld      x15, FR_X15(sp)
    ld      x16, FR_X16(sp)
    ld      x17, FR_X17(sp)
    ld      x18, FR_X18(sp)
    ld      x19, FR_X19(sp)
    ld      x20, FR_X20(sp)
    ld      x21, FR_X21(sp)
    ld      x22, FR_X22(sp)
    ld      x23, FR_X23(sp)
    ld      x24, FR_X24(sp)
    ld      x25, FR_X25(sp)
    ld      x26, FR_X26(sp)
    ld      x27, FR_X27(sp)
    ld      x28, FR_X28(sp)
    ld      x29, FR_X29(sp)
    ld      x30, FR_X30(sp)
    ld      x31, FR_X31(sp)

    /* Finally restore t0 (x5) — must be last among the GPRs. */
    ld      x5,  FR_X5(sp)

    /* Unwind the frame and swap sp <-> mscratch (which holds thread sp). */
    addi    sp, sp, 256                     /* sp = irq_sp_top */
    csrrw   sp, mscratch, sp                /* sp = thread sp, mscratch = irq_sp_top */
    mret

    /* -----------------------------------------------------------------
     * cpu_context_switch_rv64(old_tcb, new_tcb)
     *   a0 = old_tcb (may be null on first-switch)
     *   a1 = new_tcb
     *
     * Voluntary-yield switch.  Save callee-saved state + ra/sp into old,
     * load from new, jump to new's saved resume point.
     *
     * Callers treat this as a C function call, so t/a/ra regs are caller-saved
     * across the call — we don't need to preserve them in `old`.  We *do*
     * need to store the resume PC (= current ra) and sp so that the restore
     * path can enter the old thread again later.
     *
     * Design note: the restore tail is uniform with the trap-exit path (via
     * the same TCB layout).  For *newly-created* threads the scheduler
     * seeds regs[x1-1] = thread_bootstrap_rv64 and regs[x10-1] = TCB* so
     * entering the thread via `jr ra` transfers control to the bootstrap
     * with a0 = TCB pointer.
     * ----------------------------------------------------------------- */
    .global cpu_context_switch_rv64
cpu_context_switch_rv64:
    beqz    a0, .L_ctx_restore          /* first switch: skip save */

    /* Save callee-saved regs + ra + sp into old_tcb. */
    sd      ra,  (TCB_REGS_OFF + 0*8)(a0)   /* x1 */
    sd      sp,  (TCB_REGS_OFF + 1*8)(a0)   /* x2 */
    sd      sp,  (TCB_SP_OFF)(a0)           /* mirror */
    sd      s0,  (TCB_REGS_OFF + 7*8)(a0)   /* x8 */
    sd      s1,  (TCB_REGS_OFF + 8*8)(a0)   /* x9 */
    sd      s2,  (TCB_REGS_OFF + 17*8)(a0)  /* x18 */
    sd      s3,  (TCB_REGS_OFF + 18*8)(a0)
    sd      s4,  (TCB_REGS_OFF + 19*8)(a0)
    sd      s5,  (TCB_REGS_OFF + 20*8)(a0)
    sd      s6,  (TCB_REGS_OFF + 21*8)(a0)
    sd      s7,  (TCB_REGS_OFF + 22*8)(a0)
    sd      s8,  (TCB_REGS_OFF + 23*8)(a0)
    sd      s9,  (TCB_REGS_OFF + 24*8)(a0)
    sd      s10, (TCB_REGS_OFF + 25*8)(a0)
    sd      s11, (TCB_REGS_OFF + 26*8)(a0)
    /* Resume pc = ra; mstatus snapshot. */
    sd      ra,  (TCB_PC_OFF)(a0)
    csrr    t0, mstatus
    sd      t0,  (TCB_MSTATUS_OFF)(a0)

.L_ctx_restore:
    /* Load from new_tcb. */
    ld      sp,  (TCB_REGS_OFF + 1*8)(a1)
    ld      s0,  (TCB_REGS_OFF + 7*8)(a1)
    ld      s1,  (TCB_REGS_OFF + 8*8)(a1)
    ld      s2,  (TCB_REGS_OFF + 17*8)(a1)
    ld      s3,  (TCB_REGS_OFF + 18*8)(a1)
    ld      s4,  (TCB_REGS_OFF + 19*8)(a1)
    ld      s5,  (TCB_REGS_OFF + 20*8)(a1)
    ld      s6,  (TCB_REGS_OFF + 21*8)(a1)
    ld      s7,  (TCB_REGS_OFF + 22*8)(a1)
    ld      s8,  (TCB_REGS_OFF + 23*8)(a1)
    ld      s9,  (TCB_REGS_OFF + 24*8)(a1)
    ld      s10, (TCB_REGS_OFF + 25*8)(a1)
    ld      s11, (TCB_REGS_OFF + 26*8)(a1)
    /* First-entry seed: scheduler sets regs[x10-1] = TCB* so thread
     * bootstrap gets its TCB in a0. */
    ld      a0,  (TCB_REGS_OFF + 9*8)(a1)
    /* Resume pc is in the ra slot. */
    ld      ra,  (TCB_REGS_OFF + 0*8)(a1)
    ret
