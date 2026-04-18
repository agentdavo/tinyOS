/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/**
 * @file cpu_arm64.S
 * @brief ARM64 CPU specific assembly functions for miniOS v1.7.
 */

    .equ STACK_SIZE_ASM, 0x4000      // 16KB, matches CORE_STARTUP_STACK_SIZE in linker script
    .equ MAX_CORES_ASM,  4
    .equ UART_BASE,      0x09000000
    .equ UARTFR,         0x18
    .equ UARTDR,         0x00
    .equ UART_TXFF,      (1 << 5)

    // Reserve per-core boot stacks in their own section. The linker script
    // places this section in RAM so early stack pointers have valid memory.
    .section .stacks,"aw",%nobits
    .align 16
    .global boot_stacks
boot_stacks:
    .skip STACK_SIZE_ASM * 4

    // Per-core "in IRQ" flag. Set by current_el_spx_irq_entry on entry,
    // cleared on exit. Used by cpu_context_switch_impl to know whether the
    // outgoing thread's full context has already been spilled into its TCB
    // (by the IRQ entry path) — in which case the switch is a pure restore
    // handled by the IRQ exit path, and cpu_context_switch_impl returns
    // normally instead of doing its own save+eret.
    .section .bss,"aw",%nobits
    .align 3
    .global g_irq_in_progress
g_irq_in_progress:
    .skip 8 * MAX_CORES_ASM

    .text

    // TCB Offsets (ensure these match kernel::core::TCB layout in core.hpp)
    .equ TCB_REGS_X0_OFFSET,  (0*8)
    .equ TCB_REGS_X30_OFFSET, (30*8) // Offset for x30 (LR)
    .equ TCB_SP_OFFSET,       (31*8) // Offset for SP
    .equ TCB_PC_OFFSET,       (31*8 + 8)  // Offset for PC (ELR_EL1)
    .equ TCB_PSTATE_OFFSET,   (31*8 + 16) // Offset for PSTATE (SPSR_EL1)

// External symbols defined in linker script or C++/other object files
.extern __init_array_start     // Provided by linker script
.extern __init_array_end       // Provided by linker script
.extern _bss_start             // Provided by linker script
.extern _bss_end               // Provided by linker script
.extern _stacks_start          // Provided by linker script
.extern _stacks_end            // Provided by linker script (good to have if ever needed)
.extern kernel_g_per_cpu_data  // Defined in kernel_globals.cpp (via core.hpp)
.extern kernel_main            // Defined in core.cpp
.extern hal_irq_handler        // Defined in hal.cpp
.extern early_uart_puts        // Defined in freestanding_stubs.cpp
.extern early_uart_init        // Defined in freestanding_stubs.cpp
.global g_arm64_dtb_ptr

    .section .data
    .align 3
g_arm64_dtb_ptr:
    .quad 0xFFFFFFFFFFFFFFFF

    .section .rodata

.L_boot_prefix:       .asciz "[boot] "
.L_boot_core_label:   .asciz "core "
.L_boot_stage_sep:    .asciz ": "
.L_boot_hw_el:        .asciz "el="
.L_boot_hw_mpidr:     .asciz " mpidr="
.L_boot_hw_midr:     .asciz " midr="
.L_boot_stage_entry:  .asciz "cold\n"
.L_boot_stage_stack:  .asciz "stack\n"
.L_boot_stage_primary: .asciz "init\n"
.L_boot_stage_vectors: .asciz "vectors\n"
.L_boot_stage_bss:    .asciz "bss\n"
.L_boot_stage_ctors:  .asciz "ctors\n"
.L_boot_stage_kernel: .asciz "main\n"
.L_boot_stage_psci:   .asciz "psci\n"
.L_boot_stage_sp:   .asciz "sp\n"
.L_boot_stage_secondary_vectors: .asciz "secvecs\n"
.L_debug_halt:       .asciz "[halt] no current thread\n"
.L_debug_unhandled_exc: .asciz "[exc] unhandled\n"
.L_exc_esr:          .asciz " esr="
.L_exc_elr:          .asciz " elr="
.L_exc_far:          .asciz " far="
.L_exc_sp:           .asciz " sp="
.L_exc_core:         .asciz " core="
.L_exc_tcb:         .asciz " tcb="
.L_exc_nl:          .asciz "\n"

    .section .vectors, "ax", %progbits // "ax" = allocatable, executable
    .align 11 // 2KB alignment for VBAR_EL1
.global exception_vectors
exception_vectors:
    // Synchronous exception from Current EL while SP0 is selected
    b       unhandled_exception_entry_point   // Vector offset 0x000
    .align 7 // Each vector entry is 128 bytes (0x80)
    // IRQ from Current EL while SP0 is selected
    b       unhandled_exception_entry_point   // Vector offset 0x080
    .align 7
    // FIQ from Current EL while SP0 is selected
    b       unhandled_exception_entry_point   // Vector offset 0x100
    .align 7
    // SError from Current EL while SP0 is selected
    b       unhandled_exception_entry_point   // Vector offset 0x180
    .align 7
    // Synchronous exception from Current EL while SPx is selected
    b       unhandled_exception_entry_point   // Vector offset 0x200
    .align 7
    // IRQ from Current EL while SPx is selected
    b       current_el_spx_irq_entry          // Vector offset 0x280
    .align 7
    // FIQ from Current EL while SPx is selected
    b       unhandled_exception_entry_point   // Vector offset 0x300
    .align 7
    // SError from Current EL while SPx is selected
    b       unhandled_exception_entry_point   // Vector offset 0x380
    .align 7
    // Synchronous exception from Lower EL (AArch64)
    b       unhandled_exception_entry_point   // Vector offset 0x400
    .align 7
    // IRQ from Lower EL (AArch64)
    b       unhandled_exception_entry_point   // Vector offset 0x480
    .align 7
    // FIQ from Lower EL (AArch64)
    b       unhandled_exception_entry_point   // Vector offset 0x500
    .align 7
    // SError from Lower EL (AArch64)
    b       unhandled_exception_entry_point   // Vector offset 0x580
    .align 7
    // Synchronous exception from Lower EL (AArch32)
    b       unhandled_exception_entry_point   // Vector offset 0x600
    .align 7
    // IRQ from Lower EL (AArch32)
    b       unhandled_exception_entry_point   // Vector offset 0x680
    .align 7
    // FIQ from Lower EL (AArch32)
    b       unhandled_exception_entry_point   // Vector offset 0x700
    .align 7
    // SError from Lower EL (AArch32)
    b       unhandled_exception_entry_point   // Vector offset 0x780
    .align 7

    .section .text.boot, "ax", %progbits
    .global _start
_start:
    // If QEMU booted us at EL2, drop to EL1h. QEMU -machine virt -cpu max
    // starts at EL2 by default; our kernel code assumes EL1.
    mrs     x0, CurrentEL
    lsr     x0, x0, #2
    cmp     x0, #2
    b.ne    .L_at_el1_primary
    bl      drop_el2_to_el1
.L_at_el1_primary:

    // Determine current core ID early so we can set up a stack before
    // calling C functions which expect a valid stack pointer.
    mrs     x1, mpidr_el1
    and     x1, x1, #0xFF               // x1 = current core_id (Aff0)
    mov     x21, x1                     // Preserve core_id for later checks

    // Set up stack pointer for the current core
    ldr     x2, =_stacks_start          // Base address of all stacks
    mov     x3, #STACK_SIZE_ASM         // Size of one core's stack
    mul     x4, x1, x3                  // Offset = core_id * stack_size
    add     x2, x2, x4                  // Address of current core's stack bottom
    add     sp, x2, x3                  // SP = current core's stack top

    // Now the stack is valid, so early UART routines can be called safely.
    bl      early_uart_init
    ldr     x2, =g_arm64_dtb_ptr
    str     x0, [x2]
    stp     x0, x30, [sp, #-16]!        // Save x0 (DTB pointer) and LR
    mov     x19, x0                     // Preserve x0 across call
    mov     x0, x21
    adr     x1, .L_boot_stage_entry
    bl      boot_log_core_phase
    mrs     x0, CurrentEL
    adr     x1, .L_boot_hw_el
    bl      boot_log_reg64
    mrs     x0, mpidr_el1
    adr     x1, .L_boot_hw_mpidr
    bl      boot_log_reg64
    mrs     x0, midr_el1
    adr     x1, .L_boot_hw_midr
    bl      boot_log_reg64
    adr     x0, .L_exc_nl
    bl      early_uart_puts
    mov     x0, x19                     // Restore original x0
    ldp     x0, x30, [sp], #16

    mov     x0, x21                     // core_id
    adr     x1, .L_boot_stage_stack
    bl      boot_log_core_phase

    // On QEMU virt with PSCI enabled (default), only core 0 enters _start.
    // Secondary cores are parked in PSCI wait state and come up at
    // _secondary_start when core 0 calls PSCI_CPU_ON. The cbz below is
    // defensive — if a secondary somehow enters here, halt it. Test the
    // callee-saved x21 (preserved core_id); x0/x1 are caller-save and just
    // got clobbered by the bl above.
    cbz     x21, .L_primary_core_continue_setup

.L_unexpected_secondary_halt:
    wfi
    b       .L_unexpected_secondary_halt

.L_primary_core_continue_setup:
    // --- Primary Core (Core 0) Only Initialization ---
    mov     x0, x21
    adr     x1, .L_boot_stage_primary
    bl      boot_log_core_phase

    // Set up exception vector table base address
    ldr     x0, =exception_vectors
    msr     vbar_el1, x0
    isb                             // Instruction Synchronization Barrier

    mov     x0, x21
    adr     x1, .L_boot_stage_vectors
    bl      boot_log_core_phase

    // Enable Floating Point and NEON/SIMD access for EL1
    // This must be done before any C++ constructors that might use FP/NEON.
    mrs     x0, cpacr_el1
    orr     x0, x0, #(0b11 << 20)   // Set FPEN bits[21:20] to 0b11 to enable access
    msr     cpacr_el1, x0
    isb                             // Synchronize context changes

    // Zero out the BSS section before constructors run
    mov     x0, x21
    adr     x1, .L_boot_stage_bss
    bl      boot_log_core_phase

    ldr     x1, =_bss_start
    ldr     x2, =_bss_end
.L_bss_zero_loop:
    cmp     x1, x2
    b.ge    .L_bss_zero_done
    str     xzr, [x1], #8           // Store 64-bit zero and post-increment address by 8
    b       .L_bss_zero_loop
.L_bss_zero_done:

    mov     x0, x21
    adr     x1, .L_boot_stage_ctors
    bl      boot_log_core_phase
    bl      call_constructors

    // Jump to C kernel_main
    mov     x0, x21
    adr     x1, .L_boot_stage_kernel
    bl      boot_log_core_phase

    // x0 should still contain the original DTB pointer / core 0 identifier if passed by bootloader.
    // Pass it to kernel_main if your kernel_main expects it.
    // For now, assuming kernel_main takes no arguments as per typical C main.
    // If kernel_main expects (void* dtb_ptr, uint32_t core_id), set up x0, x1 here.
    bl      kernel_main

    // Fall through: eret to this core's (core 0) current_thread.
    mov     x19, #0                     // core_id for the common tail
    b       .L_eret_to_current_thread

// --- Secondary core entry (brought up by PSCI_CPU_ON) ---
// PSCI passes context_id in x0. We pass core_id as the context.
    .global _secondary_start
_secondary_start:
    // PSCI may land us at EL2 on secondary cores; drop to EL1h like core 0.
    mrs     x1, CurrentEL
    lsr     x1, x1, #2
    cmp     x1, #2
    b.ne    .L_at_el1_secondary
    bl      drop_el2_to_el1
.L_at_el1_secondary:
    mov     x19, x0                     // Preserve core_id across calls (x19 callee-saved)

    // Per-core stack inside the .stacks section (allocated by boot_stacks).
    // Must set SP BEFORE any `bl` to a C function, since function prologs
    // push x29/x30 to the stack. PSCI leaves SP undefined on entry, so
    // calling boot_log_core_phase first would crash on the first stp.
    ldr     x1, =boot_stacks
    mov     x2, #STACK_SIZE_ASM
    madd    x1, x19, x2, x1             // x1 = boot_stacks + core_id * STACK_SIZE
    add     x1, x1, x2                  // Top of this core's stack
    and     x1, x1, #~0xFUL             // 16-byte aligned
    mov     sp, x1

    mov     x0, x19
    adr     x1, .L_boot_stage_psci
    bl      boot_log_core_phase

    mov     x0, x19
    adr     x1, .L_boot_stage_sp
    bl      boot_log_core_phase

    // Install exception vectors (shared with core 0).
    ldr     x0, =exception_vectors
    msr     vbar_el1, x0
    isb
    mov     x0, x19
    adr     x1, .L_boot_stage_secondary_vectors
    bl      boot_log_core_phase

    // Enable FP/SIMD for EL1.
    mrs     x0, cpacr_el1
    orr     x0, x0, #(0b11 << 20)
    msr     cpacr_el1, x0
    isb

    // C entry: kernel_secondary_main(core_id)
    mov     x0, x19
    bl      kernel_secondary_main

    // Fall through to shared tail.
    // x19 still holds core_id.

.L_eret_to_current_thread:
    // x19 = core_id. Load g_per_cpu_data[core_id].current_thread and eret.
    ldr     x10, =kernel_g_per_cpu_data
    ldr     x10, [x10]                  // &g_per_cpu_data[0]
    mov     x11, #16                    // sizeof(PerCPUData) — two pointers
    madd    x10, x19, x11, x10          // &g_per_cpu_data[core_id]
    ldr     x0, [x10, #0]               // current_thread
    cbz     x0, .L_start_core0_halt

    ldr     x1, [x0, #TCB_SP_OFFSET]
    mov     sp, x1
    ldr     x30, [x0, #TCB_PC_OFFSET]
    ldr     x1, [x0, #TCB_PSTATE_OFFSET]
    msr     spsr_el1, x1
    // Load GPRs. After the first ldp x0,x1, x0 = regs[0] which equals &tcb (the
    // TCB pointer was stored there by create_thread). So x0 stays a valid base.
    ldp     x0, x1,  [x0, #TCB_REGS_X0_OFFSET + (0*8)]
    ldp     x2, x3,  [x0, #TCB_REGS_X0_OFFSET + (2*8)]
    ldp     x4, x5,  [x0, #TCB_REGS_X0_OFFSET + (4*8)]
    ldp     x6, x7,  [x0, #TCB_REGS_X0_OFFSET + (6*8)]
    ldp     x8, x9,  [x0, #TCB_REGS_X0_OFFSET + (8*8)]
    ldp     x10, x11,[x0, #TCB_REGS_X0_OFFSET + (10*8)]
    ldp     x12, x13,[x0, #TCB_REGS_X0_OFFSET + (12*8)]
    ldp     x14, x15,[x0, #TCB_REGS_X0_OFFSET + (14*8)]
    ldp     x16, x17,[x0, #TCB_REGS_X0_OFFSET + (16*8)]
    ldp     x18, x19,[x0, #TCB_REGS_X0_OFFSET + (18*8)]
    ldp     x20, x21,[x0, #TCB_REGS_X0_OFFSET + (20*8)]
    ldp     x22, x23,[x0, #TCB_REGS_X0_OFFSET + (22*8)]
    ldp     x24, x25,[x0, #TCB_REGS_X0_OFFSET + (24*8)]
    ldp     x26, x27,[x0, #TCB_REGS_X0_OFFSET + (26*8)]
    ldp     x28, x29,[x0, #TCB_REGS_X0_OFFSET + (28*8)]
    msr     elr_el1, x30
    eret

.L_start_core0_halt:
    // Reached only if current_thread was null (shouldn't happen once idle threads exist).
    stp     x0, x30, [sp, #-16]!
    adr     x0, .L_debug_halt
    bl      early_uart_puts
    ldp     x0, x30, [sp], #16
.L_permanent_halt_loop_core0:
    wfi
    b       .L_permanent_halt_loop_core0


    .section .text
    .align 2 // Ensure functions are aligned

.global call_constructors
call_constructors:
    stp     x19, x20, [sp, #-16]!       // Save callee-saved regs we use
    mov     x19, lr                     // Save Link Register (x30)
    ldr     x20, =__init_array_end
    ldr     x0, =__init_array_start
.L_init_loop:
    cmp     x0, x20                     // Compare current pointer with end
    b.ge    .L_init_done
    ldr     x2, [x0], #8                // Load function pointer, then x0 += 8
    cbz     x2, .L_init_loop
    // Preserve loop state (x0 = next pointer, x20 = end) across the callee,
    // which is AAPCS64 but we double-check by stashing on the stack.
    stp     x0, x20, [sp, #-16]!
    blr     x2
    ldp     x0, x20, [sp], #16
    b       .L_init_loop
.L_init_done:
    mov     lr, x19
    ldp     x19, x20, [sp], #16
    ret

// Thunk to the C implementation. boot_log_core_phase_c emits the full
// "[boot] core <hex>: <phase>" line under a single early-UART lock, so
// output from concurrent secondary cores can't interleave mid-line.
.global boot_log_core_phase
boot_log_core_phase:
    b       boot_log_core_phase_c

// Thunk to the C implementation — emits label + hex under a single lock
// so output from concurrent cores can't interleave between segments.
.global boot_log_reg64
boot_log_reg64:
    b       boot_log_reg64_c

// IRQ Handler Entry: Current EL, SPx (EL1 using SP_EL1).
//
// Design: spill the *complete* interrupted-thread state (x0-x30, pre-IRQ SP,
// ELR_EL1, SPSR_EL1) directly into the running thread's TCB. This means the
// TCB always holds the canonical "where was I" snapshot whenever a C function
// reachable from this handler runs. The C IRQ handler may invoke the
// scheduler, which may decide to switch threads simply by updating
// g_per_cpu_data[core].current_thread. On exit, we read current_thread back
// and restore from that TCB — so a context switch is just "publish a new
// current_thread pointer".
//
// This fixes a stack-leak bug in the previous design, which spilled state
// onto the interrupted thread's stack and then saved the deep-IRQ-time SP
// into TCB.sp. On every preemption, the thread's stack lost the IRQ frame +
// C call chain (hundreds of bytes), eventually overflowing and corrupting
// neighbouring memory. The classic symptom was a PC alignment fault after
// 100-200 timer ticks because a `ret` would load a stomped LR.
//
// AAPCS64 says x19-x29 are callee-saved across `bl hal_irq_handler`, but the
// previous code actively clobbered x19/x20/x21 *before* the call without
// preserving the interrupted thread's values. Saving everything into the TCB
// fixes that as a side-effect.
current_el_spx_irq_entry:
    // Free up two scratch registers without touching the interrupted thread's
    // state on the stack. We use SP_EL0 as a temporary scratch slot for x0
    // (we don't enable SP_EL0 anywhere), and tpidr_el1 as a scratch for x1.
    // tpidr_el1 isn't used by the kernel for anything else.
    msr     tpidr_el1, x0                           // stash x0
    msr     sp_el0, x1                              // stash x1

    // x0 = core_id (Aff0 of MPIDR_EL1)
    mrs     x0, mpidr_el1
    and     x0, x0, #0xFF

    // x1 = &g_per_cpu_data[core_id].current_thread
    // PerCPUData is two pointers (16 bytes); current_thread is at offset 0.
    ldr     x1, =kernel_g_per_cpu_data
    ldr     x1, [x1]                                // x1 = &g_per_cpu_data[0]
    add     x1, x1, x0, lsl #4                      // + core_id * 16
    ldr     x1, [x1]                                // x1 = current_thread (TCB*)

    // If current_thread is null we have an IRQ before the scheduler started,
    // which shouldn't happen — fall through to the unhandled handler.
    cbz     x1, unhandled_exception_entry_point

    // Save x2..x29 directly into the TCB. x0/x1 are stashed in sysregs above
    // and are restored to their interrupted values after we have a TCB ptr in
    // a different register (we use x2 below).
    stp     x2,  x3,  [x1, #TCB_REGS_X0_OFFSET + (2*8)]
    stp     x4,  x5,  [x1, #TCB_REGS_X0_OFFSET + (4*8)]
    stp     x6,  x7,  [x1, #TCB_REGS_X0_OFFSET + (6*8)]
    stp     x8,  x9,  [x1, #TCB_REGS_X0_OFFSET + (8*8)]
    stp     x10, x11, [x1, #TCB_REGS_X0_OFFSET + (10*8)]
    stp     x12, x13, [x1, #TCB_REGS_X0_OFFSET + (12*8)]
    stp     x14, x15, [x1, #TCB_REGS_X0_OFFSET + (14*8)]
    stp     x16, x17, [x1, #TCB_REGS_X0_OFFSET + (16*8)]
    stp     x18, x19, [x1, #TCB_REGS_X0_OFFSET + (18*8)]
    stp     x20, x21, [x1, #TCB_REGS_X0_OFFSET + (20*8)]
    stp     x22, x23, [x1, #TCB_REGS_X0_OFFSET + (22*8)]
    stp     x24, x25, [x1, #TCB_REGS_X0_OFFSET + (24*8)]
    stp     x26, x27, [x1, #TCB_REGS_X0_OFFSET + (26*8)]
    stp     x28, x29, [x1, #TCB_REGS_X0_OFFSET + (28*8)]
    str     x30,      [x1, #TCB_REGS_X30_OFFSET]

    // Recover the interrupted x0/x1 from sysregs and store them too.
    mrs     x2, tpidr_el1                           // original x0
    mrs     x3, sp_el0                              // original x1
    stp     x2, x3,   [x1, #TCB_REGS_X0_OFFSET + (0*8)]

    // Save SP (this is the pre-IRQ SP — HW didn't touch SP_EL1 on entry to
    // EL1h-from-EL1h), ELR_EL1, SPSR_EL1.
    mov     x2, sp
    str     x2, [x1, #TCB_SP_OFFSET]
    mrs     x2, elr_el1
    str     x2, [x1, #TCB_PC_OFFSET]
    mrs     x2, spsr_el1
    str     x2, [x1, #TCB_PSTATE_OFFSET]

    // Mark this core as "in IRQ" so cpu_context_switch_impl knows the
    // outgoing thread is already spilled and it should just return rather
    // than do its own save+eret.
    mrs     x2, mpidr_el1
    and     x2, x2, #0xFF                           // core_id again
    ldr     x3, =g_irq_in_progress
    mov     x4, #1
    str     x4, [x3, x2, lsl #3]

    // Carve out a 16-byte frame so SP stays 16-aligned through the C call.
    // We could keep using the interrupted thread's stack as our IRQ stack;
    // the IRQ frame is abandoned on exit (we restore SP from the TCB), so it
    // does not leak.
    sub     sp, sp, #16

    // Call hal_irq_handler(core_id).
    mov     x0, x2                                  // x0 = core_id
    bl      hal_irq_handler

    add     sp, sp, #16

    // Clear in-IRQ flag.
    mrs     x2, mpidr_el1
    and     x2, x2, #0xFF
    ldr     x3, =g_irq_in_progress
    str     xzr, [x3, x2, lsl #3]

    // Re-fetch current_thread (the scheduler may have swapped it in C).
    mov     x0, x2                                  // core_id
    ldr     x1, =kernel_g_per_cpu_data
    ldr     x1, [x1]
    add     x1, x1, x0, lsl #4
    ldr     x1, [x1]                                // current_thread
    cbz     x1, unhandled_exception_entry_point

    // Restore from TCB: SP, ELR_EL1, SPSR_EL1, then GPRs, then eret.
    ldr     x2, [x1, #TCB_SP_OFFSET]
    mov     sp, x2
    ldr     x2, [x1, #TCB_PC_OFFSET]
    msr     elr_el1, x2
    ldr     x2, [x1, #TCB_PSTATE_OFFSET]
    msr     spsr_el1, x2

    ldr     x30,      [x1, #TCB_REGS_X30_OFFSET]
    ldp     x28, x29, [x1, #TCB_REGS_X0_OFFSET + (28*8)]
    ldp     x26, x27, [x1, #TCB_REGS_X0_OFFSET + (26*8)]
    ldp     x24, x25, [x1, #TCB_REGS_X0_OFFSET + (24*8)]
    ldp     x22, x23, [x1, #TCB_REGS_X0_OFFSET + (22*8)]
    ldp     x20, x21, [x1, #TCB_REGS_X0_OFFSET + (20*8)]
    ldp     x18, x19, [x1, #TCB_REGS_X0_OFFSET + (18*8)]
    ldp     x16, x17, [x1, #TCB_REGS_X0_OFFSET + (16*8)]
    ldp     x14, x15, [x1, #TCB_REGS_X0_OFFSET + (14*8)]
    ldp     x12, x13, [x1, #TCB_REGS_X0_OFFSET + (12*8)]
    ldp     x10, x11, [x1, #TCB_REGS_X0_OFFSET + (10*8)]
    ldp     x8,  x9,  [x1, #TCB_REGS_X0_OFFSET + (8*8)]
    ldp     x6,  x7,  [x1, #TCB_REGS_X0_OFFSET + (6*8)]
    ldp     x4,  x5,  [x1, #TCB_REGS_X0_OFFSET + (4*8)]
    ldp     x2,  x3,  [x1, #TCB_REGS_X0_OFFSET + (2*8)]
    // x0,x1 last so we don't lose the TCB base before reading them.
    ldp     x0,  x1,  [x1, #TCB_REGS_X0_OFFSET + (0*8)]
    eret

// Generic unhandled exception handler. Prints a labelled ESR/ELR/FAR/SP/core/TCB
// line and halts. FAR_EL1 matters for translation faults; TCB pointer lets you
// cross-reference g_task_tcbs[] in the ELF to identify which thread faulted —
// addr2line on ELR plus the TCB's .name field tells you where and who.
    .global unhandled_exception_entry_point
unhandled_exception_entry_point:
    // Don't clobber SP_EL1 — we want to dump the interrupted SP verbatim.
    // Stash what we need into callee-saved regs (we're about to halt anyway).
    mrs     x25, esr_el1
    mrs     x26, elr_el1
    mrs     x27, far_el1
    mov     x28, sp                 // SP at exception entry
    mrs     x20, mpidr_el1
    and     x20, x20, #0xFF         // core_id

    // Look up current_thread for this core (NULL-safe).
    ldr     x21, =kernel_g_per_cpu_data
    ldr     x21, [x21]
    add     x21, x21, x20, lsl #4   // &g_per_cpu_data[core]
    ldr     x21, [x21]              // current_thread (may be NULL)

    adr     x0, .L_debug_unhandled_exc
    bl      early_uart_puts

    adr     x0, .L_exc_esr
    bl      early_uart_puts
    mov     x0, x25
    bl      uart_put_hex16
    adr     x0, .L_exc_elr
    bl      early_uart_puts
    mov     x0, x26
    bl      uart_put_hex16
    adr     x0, .L_exc_far
    bl      early_uart_puts
    mov     x0, x27
    bl      uart_put_hex16
    adr     x0, .L_exc_sp
    bl      early_uart_puts
    mov     x0, x28
    bl      uart_put_hex16
    adr     x0, .L_exc_core
    bl      early_uart_puts
    mov     x0, x20
    bl      uart_put_hex16
    adr     x0, .L_exc_tcb
    bl      early_uart_puts
    mov     x0, x21
    bl      uart_put_hex16
    adr     x0, .L_exc_nl
    bl      early_uart_puts

.L_unhandled_exception_spin_loop:
    wfi
    b       .L_unhandled_exception_spin_loop

// --- EL2 → EL1 drop. Called via `bl`; returns to its caller now at EL1h. ---
// Minimal: set HCR_EL2.RW=1 (AArch64 at EL1), SCTLR_EL1 to architectural reset
// value, and eret with SPSR_EL2 = EL1h masking Serror / debug / FIQ but NOT IRQ
// (our thread pstate will unmask IRQs once we get to the scheduled thread).
drop_el2_to_el1:
    // HCR_EL2.RW (bit 31) = 1 → AArch64 execution at EL1
    mov     x0, #(1 << 31)
    msr     hcr_el2, x0

    // CNTHCTL_EL2: EL1PCEN (bit 1), EL1PCTEN (bit 0) so EL1 can use the
    // physical timer/counter without trapping.
    mrs     x0, cnthctl_el2
    orr     x0, x0, #3
    msr     cnthctl_el2, x0
    msr     cntvoff_el2, xzr

    // SCTLR_EL1: architectural reset value. MMU off, caches off, RES1 bits set.
    mov     x0, #0x0800
    movk    x0, #0x30d0, lsl #16
    msr     sctlr_el1, x0

    // SPSR_EL2 = EL1h, DAIF all masked (we'll unmask in-thread).
    mov     x0, #0x3c5      // 0011 1100 0101 : D=1, A=1, I=1, F=1, M=0b0101
    msr     spsr_el2, x0

    // ELR_EL2 = our caller's return address (LR). After eret, control
    // resumes at EL1h right after the `bl drop_el2_to_el1`.
    msr     elr_el2, x30
    eret

// --- Tiny exception-time UART helpers (no stack beyond caller-saved regs) ---
// Write x0's low byte to UART DR, polling UART FR for TXFF first.
uart_putc_x0:
    ldr     x9, =UART_BASE
.L_uart_wait:
    ldr     w10, [x9, #UARTFR]
    tst     w10, #UART_TXFF
    b.ne    .L_uart_wait
    strb    w0, [x9, #UARTDR]
    ret

// Write x0 as 16 hex digits to UART. Preserves x0/x30 via stack. Uses x9-x12.
uart_put_hex16:
    stp     x0, x30, [sp, #-16]!
    stp     x19, x20, [sp, #-16]!
    mov     x19, x0                 // value
    mov     x20, #60                // shift
.L_hex_loop:
    lsr     x0, x19, x20
    and     x0, x0, #0xF
    cmp     x0, #10
    b.lt    1f
    add     x0, x0, #('A' - 10)
    b       2f
1:  add     x0, x0, #'0'
2:  bl      uart_putc_x0
    cbz     x20, .L_hex_done
    sub     x20, x20, #4
    b       .L_hex_loop
.L_hex_done:
    ldp     x19, x20, [sp], #16
    ldp     x0, x30, [sp], #16
    ret
    b       .L_unhandled_exception_spin_loop


// Context Switch Implementation
// Called from C: void cpu_context_switch_impl(TCB* old_tcb, TCB* new_tcb);
// x0 = old_tcb, x1 = new_tcb
//
// Two distinct callers / contexts:
//
//  (A) Preemptive (from IRQ): hal_irq_handler -> preemptive_tick -> schedule
//      -> cpu_context_switch -> cpu_context_switch_impl.
//      The IRQ entry path (current_el_spx_irq_entry) already spilled the full
//      interrupted context into old_tcb. The IRQ exit path will restore from
//      g_per_cpu_data[core].current_thread, which the C scheduler has already
//      pointed at new_tcb. So here we have nothing to do — just return.
//      g_irq_in_progress[core] is non-zero in this case.
//
//  (B) Voluntary (from thread context, e.g. yield()): there was no exception,
//      so ELR_EL1/SPSR_EL1 hold stale values and cannot be used. We must save
//      the caller's resumption state ourselves: PC = lr (return addr), SP =
//      current sp, PSTATE = synthesised "EL1h, IRQs enabled" (0x5), and the
//      callee-saved GPRs (AAPCS64: x19-x29). For safety we save x0-x29+x30
//      so the restore path can be uniform.
//      g_irq_in_progress[core] is zero in this case.
.global cpu_context_switch_impl
cpu_context_switch_impl:
    // Detect mode: IRQ vs voluntary.
    mrs     x9, mpidr_el1
    and     x9, x9, #0xFF                           // core_id
    ldr     x10, =g_irq_in_progress
    ldr     x11, [x10, x9, lsl #3]
    cbnz    x11, .L_ctx_switch_irq_path

    // ---- (B) Voluntary path: save old, restore new, eret ----
    // If old_tcb is null (shouldn't happen on the voluntary path, but keep
    // for safety / first-switch fallback) skip the save.
    cbz     x0, .L_restore_new_context_for_switch

    // Save GPRs into old_tcb (x0 base). stp/str only read the base, so we
    // can use x0 as the base while also storing x0's value.
    stp     x0,  x1,  [x0, #TCB_REGS_X0_OFFSET + (0*8)]
    stp     x2,  x3,  [x0, #TCB_REGS_X0_OFFSET + (2*8)]
    stp     x4,  x5,  [x0, #TCB_REGS_X0_OFFSET + (4*8)]
    stp     x6,  x7,  [x0, #TCB_REGS_X0_OFFSET + (6*8)]
    stp     x8,  x9,  [x0, #TCB_REGS_X0_OFFSET + (8*8)]
    stp     x10, x11, [x0, #TCB_REGS_X0_OFFSET + (10*8)]
    stp     x12, x13, [x0, #TCB_REGS_X0_OFFSET + (12*8)]
    stp     x14, x15, [x0, #TCB_REGS_X0_OFFSET + (14*8)]
    stp     x16, x17, [x0, #TCB_REGS_X0_OFFSET + (16*8)]
    stp     x18, x19, [x0, #TCB_REGS_X0_OFFSET + (18*8)]
    stp     x20, x21, [x0, #TCB_REGS_X0_OFFSET + (20*8)]
    stp     x22, x23, [x0, #TCB_REGS_X0_OFFSET + (22*8)]
    stp     x24, x25, [x0, #TCB_REGS_X0_OFFSET + (24*8)]
    stp     x26, x27, [x0, #TCB_REGS_X0_OFFSET + (26*8)]
    stp     x28, x29, [x0, #TCB_REGS_X0_OFFSET + (28*8)]
    str     lr,       [x0, #TCB_REGS_X30_OFFSET]

    // PC = lr (resume right after the C call to cpu_context_switch).
    str     lr, [x0, #TCB_PC_OFFSET]
    // PSTATE = EL1h, DAIF unmasked.
    mov     x2, #0x5
    str     x2, [x0, #TCB_PSTATE_OFFSET]
    // SP = current SP at entry.
    mov     x2, sp
    str     x2, [x0, #TCB_SP_OFFSET]

.L_restore_new_context_for_switch:
    // x1 = new_tcb. Restore SP, ELR_EL1, SPSR_EL1, GPRs, then eret.
    ldr     x2, [x1, #TCB_SP_OFFSET]
    mov     sp, x2
    ldr     x2, [x1, #TCB_PC_OFFSET]
    msr     elr_el1, x2
    ldr     x2, [x1, #TCB_PSTATE_OFFSET]
    msr     spsr_el1, x2

    ldr     x30,      [x1, #TCB_REGS_X30_OFFSET]
    ldp     x28, x29, [x1, #TCB_REGS_X0_OFFSET + (28*8)]
    ldp     x26, x27, [x1, #TCB_REGS_X0_OFFSET + (26*8)]
    ldp     x24, x25, [x1, #TCB_REGS_X0_OFFSET + (24*8)]
    ldp     x22, x23, [x1, #TCB_REGS_X0_OFFSET + (22*8)]
    ldp     x20, x21, [x1, #TCB_REGS_X0_OFFSET + (20*8)]
    ldp     x18, x19, [x1, #TCB_REGS_X0_OFFSET + (18*8)]
    ldp     x16, x17, [x1, #TCB_REGS_X0_OFFSET + (16*8)]
    ldp     x14, x15, [x1, #TCB_REGS_X0_OFFSET + (14*8)]
    ldp     x12, x13, [x1, #TCB_REGS_X0_OFFSET + (12*8)]
    ldp     x10, x11, [x1, #TCB_REGS_X0_OFFSET + (10*8)]
    ldp     x8,  x9,  [x1, #TCB_REGS_X0_OFFSET + (8*8)]
    ldp     x6,  x7,  [x1, #TCB_REGS_X0_OFFSET + (6*8)]
    ldp     x4,  x5,  [x1, #TCB_REGS_X0_OFFSET + (4*8)]
    ldp     x2,  x3,  [x1, #TCB_REGS_X0_OFFSET + (2*8)]
    ldp     x0,  x1,  [x1, #TCB_REGS_X0_OFFSET + (0*8)]
    eret

    // ---- (A) IRQ-context path: nothing to do here. ----
    // The interrupted thread is already saved into old_tcb by the IRQ entry,
    // and the IRQ exit will restore from g_per_cpu_data[core].current_thread
    // (which the scheduler updated to new_tcb before calling us). Just
    // return so the call chain unwinds back into hal_irq_handler.
.L_ctx_switch_irq_path:
    ret
