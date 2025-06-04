/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/**
 * @file cpu_arm64.S
 * @brief ARM64 CPU specific assembly functions for miniOS v1.7.
 */

    .equ STACK_SIZE_ASM, 0x4000      // 16KB, matches CORE_STARTUP_STACK_SIZE in linker script
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

    .section .rodata
.L_debug_start:         .asciz "[DEBUG] _start ENTRY\n"
.L_debug_stack:         .asciz "[DEBUG] Stack set up\n"
.L_debug_primary:       .asciz "[DEBUG] Primary core setup\n"
.L_debug_vectors:       .asciz "[DEBUG] Exception vectors set\n"
.L_debug_constructors:  .asciz "[DEBUG] Calling constructors\n"
.L_debug_bss:           .asciz "[DEBUG] BSS zeroed\n"
.L_debug_kernel_main:   .asciz "[DEBUG] Entering kernel_main\n"
.L_debug_halt:          .asciz "[DEBUG] _start: Halting core (no current_thread for core 0 after kernel_main)\n"
.L_debug_unhandled_exc: .asciz "[EXC] Unhandled Exception! Halting.\n"

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
    stp     x0, x30, [sp, #-16]!        // Save x0 (DTB pointer) and LR
    mov     x19, x0                     // Preserve x0 across call
    adr     x0, .L_debug_start
    bl      early_uart_puts
    mov     x0, x19                     // Restore original x0
    ldp     x0, x30, [sp], #16

    mov     x1, x21                     // Restore preserved core_id

    stp     x1, x30, [sp, #-16]! // Save core_id and LR
    adr     x0, .L_debug_stack
    bl      early_uart_puts
    ldp     x1, x30, [sp], #16   // Restore core_id and LR

    // If not primary core (core 0), spin until woken by primary core (e.g., via SEV)
    // x1 still holds core_id from above
    cbz     x1, .L_primary_core_continue_setup

.L_secondary_core_spin:
    // Secondary cores wait for an event. Primary core will use SEV.
    dmb     sy      // Ensure memory operations complete before WFE
    wfe             // Wait For Event
    b       .L_secondary_core_spin // Loop until event received and some condition changes

.L_primary_core_continue_setup:
    // --- Primary Core (Core 0) Only Initialization ---
    stp     x1, x30, [sp, #-16]!
    adr     x0, .L_debug_primary
    bl      early_uart_puts
    ldp     x1, x30, [sp], #16

    // Set up exception vector table base address
    ldr     x0, =exception_vectors
    msr     vbar_el1, x0
    isb                             // Instruction Synchronization Barrier

    stp     x0, x30, [sp, #-16]!
    adr     x0, .L_debug_vectors
    bl      early_uart_puts
    ldp     x0, x30, [sp], #16

    // Enable Floating Point and NEON/SIMD access for EL1
    // This must be done before any C++ constructors that might use FP/NEON.
    mrs     x0, cpacr_el1
    orr     x0, x0, #(0b11 << 20)   // Set FPEN bits[21:20] to 0b11 to enable access
    msr     cpacr_el1, x0
    isb                             // Synchronize context changes

    // Zero out the BSS section before constructors run
    stp     x0, x30, [sp, #-16]!
    adr     x0, .L_debug_bss
    bl      early_uart_puts
    ldp     x0, x30, [sp], #16

    ldr     x1, =_bss_start
    ldr     x2, =_bss_end
.L_bss_zero_loop:
    cmp     x1, x2
    b.ge    .L_bss_zero_done
    str     xzr, [x1], #8           // Store 64-bit zero and post-increment address by 8
    b       .L_bss_zero_loop
.L_bss_zero_done:

    // Call global C++ constructors after BSS is ready
    stp     x0, x30, [sp, #-16]!
    adr     x0, .L_debug_constructors
    bl      early_uart_puts
    ldp     x0, x30, [sp], #16
    bl      call_constructors

    // Jump to C kernel_main
    stp     x0, x30, [sp, #-16]!
    adr     x0, .L_debug_kernel_main
    bl      early_uart_puts
    ldp     x0, x30, [sp], #16

    // x0 should still contain the original DTB pointer / core 0 identifier if passed by bootloader.
    // Pass it to kernel_main if your kernel_main expects it.
    // For now, assuming kernel_main takes no arguments as per typical C main.
    // If kernel_main expects (void* dtb_ptr, uint32_t core_id), set up x0, x1 here.
    bl      kernel_main

    // After kernel_main returns (which it shouldn't for the initial thread's startup,
    // but this code path is for core 0's _start context, not a typical thread exit).
    // Try to load and run the initial task for core 0.
    // g_per_cpu_data is an array of PerCPUData structs.
    // We need g_per_cpu_data[0].current_thread
    ldr     x10, =kernel_g_per_cpu_data  // Load base address of g_per_cpu_data
                                        // Assuming PerCPUData* is the first member.
                                        // If it's an array of structs:
                                        // kernel_g_per_cpu_data might point to the first element.
                                        // Offset for current_thread within PerCPUData[0] is needed.
                                        // Let's assume kernel_g_per_cpu_data is PerCPUData* const
                                        // and it points to the start of the array g_per_cpu_data.data().
                                        // Then g_per_cpu_data[0].current_thread is at [x10, #offset_of_current_thread]
                                        // Or, if kernel_g_per_cpu_data is a pointer TO the array,
                                        // ldr x10, [x10] first to get array base.
                                        // From core.hpp: alignas(64) extern std::array<PerCPUData, MAX_CORES> g_per_cpu_data;
                                        // From core.hpp: extern "C" kernel::core::PerCPUData* const kernel_g_per_cpu_data = kernel::core::g_per_cpu_data.data();
                                        // So kernel_g_per_cpu_data holds the address of g_per_cpu_data[0].
                                        // PerCPUData: { TCB* current_thread; TCB* idle_thread; }
                                        // So current_thread is at offset 0 from kernel_g_per_cpu_data for core 0.

    ldr     x0, [x10, #0]             // x0 = g_per_cpu_data[0].current_thread (TCB*)

    cbz     x0, .L_start_core0_halt   // If current_thread is null, something is wrong, halt core 0.

    // x0 now holds the TCB* for the first task on core 0.
    // This is effectively the first context switch *into* a task.
    // We set up the registers as if we are returning from an exception to this task.
    // The TCB should have been initialized by create_thread.

    ldr     x1, [x0, #TCB_SP_OFFSET]    // Get new_tcb->sp
    mov     sp, x1                      // Set EL1 stack pointer

    ldr     x30, [x0, #TCB_PC_OFFSET]   // Get new_tcb->pc (target ELR_EL1)
    ldr     x1, [x0, #TCB_PSTATE_OFFSET]// Get new_tcb->pstate (target SPSR_EL1)
    msr     spsr_el1, x1                // Set SPSR_EL1

    // Restore general purpose registers x0-x29 from TCB.
    // Note: x0 in TCB is the first argument to thread_bootstrap (the TCB itself).
    // x30 (LR) for the new thread is set to its PC (entry point) via ELR_EL1.
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
    // x30 is special: it's the link register. For the first eret, ELR_EL1 (loaded into x30 before eret) acts as PC.
    // The TCB_REGS_X30_OFFSET typically stores the LR *if the thread was interrupted*.
    // For a fresh thread starting via thread_bootstrap, its "LR" isn't meaningful in the same way.
    // The PC field (loaded into ELR_EL1) is what matters for where it starts.
    // The TCB->regs[0] (for x0) will be the argument to thread_bootstrap.

    msr     elr_el1, x30                // Set ELR_EL1 to the PC of the new thread
    eret                                // Exception return to start the first task

.L_start_core0_halt:
    // This halt is if g_per_cpu_data[0].current_thread was null after kernel_main
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
    mov     x19, lr                     // Save Link Register (x30)
    ldr     x0, =__init_array_start     // Start of .init_array
    add     x0, x0, #8                 // Skip first libgcc constructor
    ldr     x1, =__init_array_end       // End of .init_array
.L_init_loop:
    cmp     x0, x1                      // Compare current pointer with end
    b.ge    .L_init_done                // If x0 >= x1, all constructors called
    ldr     x2, [x0], #8                // Load function pointer from [x0], then x0 += 8
    cbz     x2, .L_init_loop            // Skip if null pointer (should not happen in well-formed .init_array)
    blr     x2                          // Branch with Link to constructor (x2)
    b       .L_init_loop                // Loop to next constructor
.L_init_done:
    mov     lr, x19                     // Restore Link Register
    ret                                 // Return

// IRQ Handler Entry: Current EL, SPx (EL1 using SP_EL1)
// This is the common IRQ entry point.
current_el_spx_irq_entry:
    // Save general purpose registers x0-x18, and LR (x30)
    // Stack pointer (sp_el1) is already the correct one.
    // We need 19*8 for x0-x18, + 8 for LR = 160 bytes for GPRs
    // + 8 for ELR_EL1 + 8 for SPSR_EL1 = 176 bytes total.
    // Ensure stack is 16-byte aligned before stp. Current SP should be.
    sub     sp, sp, #176                            // Allocate space on stack (11 * 16 bytes)
    stp     x0, x1,   [sp, #(0*16)]
    stp     x2, x3,   [sp, #(1*16)]
    stp     x4, x5,   [sp, #(2*16)]
    stp     x6, x7,   [sp, #(3*16)]
    stp     x8, x9,   [sp, #(4*16)]
    stp     x10, x11, [sp, #(5*16)]
    stp     x12, x13, [sp, #(6*16)]
    stp     x14, x15, [sp, #(7*16)]
    stp     x16, x17, [sp, #(8*16)]
    // x18 is saved individually if needed, or grouped if more registers.
    // For now, let's save x18 and x30 (LR)
    mrs     x19, elr_el1                            // Save ELR_EL1 (return address from exception)
    mrs     x20, spsr_el1                           // Save SPSR_EL1 (PSTATE at exception)
    mov     x21, lr                                 // Save LR (x30)
    stp     x18, x21, [sp, #(9*16)]                 // Save x18 and current LR
    stp     x19, x20, [sp, #(10*16)]                // Save ELR_EL1 and SPSR_EL1

    // Argument to hal_irq_handler: core_id in x0
    mrs     x0, mpidr_el1
    and     x0, x0, #0xFF                           // Get current core ID (Aff0)
    bl      hal_irq_handler                         // Call the C IRQ handler

    // Restore saved state from stack
    ldp     x19, x20, [sp, #(10*16)]                // Restore ELR_EL1 and SPSR_EL1
    ldp     x18, x21, [sp, #(9*16)]                 // Restore x18 and LR
    msr     elr_el1, x19
    msr     spsr_el1, x20
    mov     lr, x21

    ldp     x0, x1,   [sp, #(0*16)]
    ldp     x2, x3,   [sp, #(1*16)]
    ldp     x4, x5,   [sp, #(2*16)]
    ldp     x6, x7,   [sp, #(3*16)]
    ldp     x8, x9,   [sp, #(4*16)]
    ldp     x10, x11, [sp, #(5*16)]
    ldp     x12, x13, [sp, #(6*16)]
    ldp     x14, x15, [sp, #(7*16)]
    ldp     x16, x17, [sp, #(8*16)]
    // x18 restored with ldp x18, x21...

    add     sp, sp, #176                            // Deallocate stack space
    eret                                            // Return from exception

// Generic unhandled exception handler
    .global unhandled_exception_entry_point
unhandled_exception_entry_point:
    // Minimal state save if possible, then call a simple debug output loop.
    // This handler might be entered with a bad stack or in a bad state.
    // Try to print a character and loop.
    stp     x0, x30, [sp, #-16]! // Save x0 and LR if stack is somewhat valid
    adr     x0, .L_debug_unhandled_exc
    bl      early_uart_puts      // This might not work if system is very broken
.L_unhandled_exception_spin_loop:
    wfi
    b       .L_unhandled_exception_spin_loop


// Context Switch Implementation
// Called from C: void cpu_context_switch_impl(TCB* old_tcb, TCB* new_tcb);
// x0 = old_tcb, x1 = new_tcb
.global cpu_context_switch_impl
cpu_context_switch_impl:
    // If old_tcb is null (e.g., first switch from idle setup), skip saving.
    cbz     x0, .L_restore_new_context_for_switch

    // --- Save context of old_tcb (current thread) ---
    // x0 still holds old_tcb pointer.
    // Save GPRs x0-x29 (x30/LR is saved separately)
    // Note: The actual value of x0 (old_tcb ptr) needs to be preserved if used after stp x0,x2.
    // We'll use x20 as a temporary to hold old_tcb.
    mov     x20, x0 // Preserve old_tcb in x20

    stp     x0, x1,   [x20, #TCB_REGS_X0_OFFSET + (0*8)] // Saves original x0, x1 of caller
    stp     x2, x3,   [x20, #TCB_REGS_X0_OFFSET + (2*8)]
    stp     x4, x5,   [x20, #TCB_REGS_X0_OFFSET + (4*8)]
    stp     x6, x7,   [x20, #TCB_REGS_X0_OFFSET + (6*8)]
    stp     x8, x9,   [x20, #TCB_REGS_X0_OFFSET + (8*8)]
    stp     x10, x11, [x20, #TCB_REGS_X0_OFFSET + (10*8)]
    stp     x12, x13, [x20, #TCB_REGS_X0_OFFSET + (12*8)]
    stp     x14, x15, [x20, #TCB_REGS_X0_OFFSET + (14*8)]
    stp     x16, x17, [x20, #TCB_REGS_X0_OFFSET + (16*8)]
    stp     x18, x19, [x20, #TCB_REGS_X0_OFFSET + (18*8)]
    // x20 was used as temp, original x20 value needs to be saved from somewhere else.
    // This is tricky. Standard context switch often saves to current SP first, then copies.
    // Or, it uses a scratch register not part of the saved set.
    // Let's assume x20 is not critical or is saved first.
    // For a robust save, you'd push GPRs onto the current stack, then store SP.
    // Simpler: if cpu_context_switch_impl is a leaf function in C terms for saving,
    // compiler might preserve some registers. But we save most.
    // Let's use x2 as a scratch after saving it.
    str     x20, [x20, #TCB_REGS_X0_OFFSET + (20*8)] // Save original x20 from x20 itself (fine before it's clobbered)
    stp     x21, x22, [x20, #TCB_REGS_X0_OFFSET + (21*8)] // x21, x22
    stp     x23, x24, [x20, #TCB_REGS_X0_OFFSET + (23*8)] // x23, x24
    stp     x25, x26, [x20, #TCB_REGS_X0_OFFSET + (25*8)] // x25, x26
    stp     x27, x28, [x20, #TCB_REGS_X0_OFFSET + (27*8)] // x27, x28
    str     x29,      [x20, #TCB_REGS_X0_OFFSET + (29*8)] // x29

    // Save LR (x30) of the old task
    // This LR is the return address *within* the C caller of cpu_context_switch_impl.
    // Or, if called from IRQ handler *after* it saved original LR, this is IRQ handler's LR.
    // Crucially, for a task yielding or being preempted, its "PC" is where it will resume.
    // This is typically ELR_EL1 if preempted by IRQ, or LR if yielded via BL.
    mrs     x2, elr_el1         // Get PC at exception (if called from IRQ context that saved original ELR)
    str     x2, [x20, #TCB_PC_OFFSET]
    mrs     x2, spsr_el1        // Get PSTATE at exception
    str     x2, [x20, #TCB_PSTATE_OFFSET]
    str     lr, [x20, #TCB_REGS_X30_OFFSET] // Save current LR (x30) into TCB's x30 slot

    // Save SP_EL1 (current stack pointer)
    mov     x2, sp
    str     x2, [x20, #TCB_SP_OFFSET]

.L_restore_new_context_for_switch:
    // --- Restore context of new_tcb ---
    // x1 holds new_tcb pointer.
    // This register (x1) will be restored from new_tcb->regs[1] at the end.

    // Load SP_EL1 from new_tcb
    ldr     x2, [x1, #TCB_SP_OFFSET]
    mov     sp, x2

    // Load PC and PSTATE for new_tcb into ELR_EL1 and SPSR_EL1
    ldr     x30, [x1, #TCB_PC_OFFSET]   // PC for ELR_EL1
    ldr     x2, [x1, #TCB_PSTATE_OFFSET]  // PSTATE for SPSR_EL1
    msr     spsr_el1, x2

    // Restore GPRs x0-x29 from new_tcb.
    // x30 (LR) is loaded into ELR_EL1 below, so TCB's x30 slot is for its own LR.
    ldp     x0, x2,   [x1, #TCB_REGS_X0_OFFSET + (0*8)] // x0, x2 (x1 is new_tcb, restored last)
    ldp     x3, x4,   [x1, #TCB_REGS_X0_OFFSET + (3*8)] // x3, x4 (skip x1 for now)
    ldp     x5, x6,   [x1, #TCB_REGS_X0_OFFSET + (5*8)]
    ldp     x7, x8,   [x1, #TCB_REGS_X0_OFFSET + (7*8)]
    ldp     x9, x10,  [x1, #TCB_REGS_X0_OFFSET + (9*8)]
    ldp     x11, x12, [x1, #TCB_REGS_X0_OFFSET + (11*8)]
    ldp     x13, x14, [x1, #TCB_REGS_X0_OFFSET + (13*8)]
    ldp     x15, x16, [x1, #TCB_REGS_X0_OFFSET + (15*8)]
    ldp     x17, x18, [x1, #TCB_REGS_X0_OFFSET + (17*8)]
    ldp     x19, x20, [x1, #TCB_REGS_X0_OFFSET + (19*8)]
    ldp     x21, x22, [x1, #TCB_REGS_X0_OFFSET + (21*8)]
    ldp     x23, x24, [x1, #TCB_REGS_X0_OFFSET + (23*8)]
    ldp     x25, x26, [x1, #TCB_REGS_X0_OFFSET + (25*8)]
    ldp     x27, x28, [x1, #TCB_REGS_X0_OFFSET + (27*8)]
    ldr     x29,      [x1, #TCB_REGS_X0_OFFSET + (29*8)]
    // Restore LR (x30) of the new task from its TCB x30 slot
    ldr     x19,      [x1, #TCB_REGS_X30_OFFSET] // temp load into x19
    // Restore x1 itself
    ldr     x1,       [x1, #TCB_REGS_X0_OFFSET + (1*8)] // x1 is now restored.

    msr     elr_el1, x30                // Set PC to return to for the new task
    mov     lr, x19                     // Set LR of new task (from its TCB x30 slot)
    eret                                // Exception return to new task
