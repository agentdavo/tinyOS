// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file cpu_arm64.S
 * @brief ARM64 CPU specific assembly functions for miniOS v1.7.
 */

    .equ STACK_SIZE_ASM, 0x4000   // Must match STACK_SIZE in linker script

    // Constants for accessing the UART (PL011)
    .equ UART_BASE, 0x09000000
    .equ UARTFR,    0x18
    .equ UARTDR,    0x00
    .equ UART_TXFF, (1 << 5)  // Transmit FIFO Full flag

    // TCB Offsets
    .equ TCB_REGS_X0_OFFSET, (0*8)
    .equ TCB_REGS_X30_OFFSET, (30*8)
    .equ TCB_SP_OFFSET, (31*8)      
    .equ TCB_PC_OFFSET, (31*8 + 8)  
    .equ TCB_PSTATE_OFFSET, (31*8 + 16) 

    .section .vectors, "ax", %progbits
    .align 11
.global exception_vectors 
exception_vectors:
    b       unhandled_exception_sp0_sync; .align 7 
    b       unhandled_exception_sp0_irq;  .align 7
    b       unhandled_exception_sp0_fiq;  .align 7
    b       unhandled_exception_sp0_serror; .align 7
    b       unhandled_exception_spx_sync; .align 7
    b       current_el_spx_irq_entry; .align 7 
    b       unhandled_exception_spx_fiq;  .align 7
    b       unhandled_exception_spx_serror; .align 7
    b       unhandled_exception_lower_a64_sync; .align 7
    b       unhandled_exception_lower_a64_irq;  .align 7
    b       unhandled_exception_lower_a64_fiq;  .align 7
    b       unhandled_exception_lower_a64_serror; .align 7
    b       unhandled_exception_lower_a32_sync; .align 7
    b       unhandled_exception_lower_a32_irq;  .align 7
    b       unhandled_exception_lower_a32_fiq;  .align 7
    b       unhandled_exception_lower_a32_serror; .align 7

    .section .text.boot, "ax", %progbits
    .global _start  
_start:
    // 0. Get core ID
    mrs     x0, mpidr_el1
    and     x0, x0, #0xFF          // x0 = current core_id

    // 1. Set up stack for this core *before* global init if SMP to prevent any collisions
    ldr     x1, =_stacks_start          // Linker provides this, base of stack array
    mov     x2, #STACK_SIZE_ASM         // One stack per core
    mul     x3, x0, x2                  // offset = core_id * STACK_SIZE_ASM
    add     x1, x1, x3                  // base + offset = stack location for this core
    add     x1, x1, x2                  // set SP to the high end of memory just allocated
    mov     sp, x1 

    // Secondary cores spin here - let primay core set up before other work
    cbz     x0, .L_primary_core_setup  // core 0 (primary) should not wait!
.L_secondary_core_spin:
    // If stack check fails and this branches, loop forever.
    dmb     sy // Make sure the spin lock variable written by .L_release_core is visible
    wfe        // Wait for Event – low power state until SEV is signaled by primary core
    b        .L_secondary_core_spin // Spin for now TODO implement flags

.L_primary_core_setup:
    ldr     x0, =exception_vectors 
    msr     vbar_el1, x0 
    isb                                 // Synchronize context after setting VBAR_EL1

    // Before calling call_constructors, lets attempt to write a simple print and check again
    stp     x0, x1, [sp, #-16]!          // push x0, x1
    ldr     x0, =UART_BASE
    mov     w1, #'P' // This code point should write "P" to the serial console.
    // Lets code a test version with the bit bashing.
.uart_debug_loop:
    ldr     w2, [x0, #UARTFR]          // Load Flag Register
    tst     w2, #UART_TXFF           // Test Transmit FIFO Full bit
    b.ne     .uart_debug_loop          // Wait for UART to be ready (TF

    str     w1, [x0, #UARTDR]          // Send character to UART
    ldp     x0, x1, [sp], #16           // pop x0, x1
    
    bl      call_constructors
	
.L_primary_core_after_static_inits:

    // Initialize .bss to zero
    ldr     x1, =_bss_start      // Load start of .bss section (linker symbols)
    ldr     x2, =_bss_end       // Load end of .bss section (linker symbols)

.L_zero_bss_loop:
    cmp     x1, x2                 // Check if we reached end of BSS
    b.ge    .L_zero_bss_done
    str     xzr, [x1], #8       // Zero 8 bytes at a time, post-increment
    b       .L_zero_bss_loop
.L_zero_bss_done:

    // Now jump into the C++ kernel main
    bl      kernel_main

.L_kernel_halt_loop:  // Dummy destination in case main returns; Should never reach
    wfi 
    b       .L_kernel_halt_loop
    // --- End of _start code ---

    .section .text
    .align 2

// The routines for printing to the screen have been inlined in code

// Call all contructors for static C++ objects - should follow ARM C++ standard

.global call_constructors
call_constructors:
    mov x19, lr
    ldr x0, =__init_array_start
    ldr x1, =__init_array_end
    // Loop to invoke all C++ ctors
.L_init_loop:
    cmp x0, x1
    b.ge .L_init_done
    ldr x2, [x0], #8
    cbz x2, .L_init_loop
    blr x2 
    b .L_init_loop
.L_init_done:
    mov lr, x19
    ret

// Dummy IRQ handler entry - Placeholder to be filled later (Exception Level 1)
current_el_spx_irq_entry:
   // Code: Save/restore registers on stack + bl hal_irq_handler.
   // For now spin for easier bring up
    b unhandled_exception_loop

unhandled_exception_loop: // Put exceptions here that do not have an address

    wfi 
    b       unhandled_exception_loop

// All exception handlers just loop and wait to be discovered via JTAG
unhandled_exception_sp0_sync:  b unhandled_exception_loop
unhandled_exception_sp0_irq:   b unhandled_exception_loop
unhandled_exception_sp0_fiq:  b unhandled_exception_loop
unhandled_exception_sp0_serror:  b unhandled_exception_loop
unhandled_exception_spx_sync:  b unhandled_exception_loop
unhandled_exception_spx_fiq:   b unhandled_exception_loop
unhandled_exception_spx_serror: b unhandled_exception_loop
unhandled_exception_lower_a64_sync: b unhandled_exception_loop
unhandled_exception_lower_a64_irq:  b unhandled_exception_loop
unhandled_exception_lower_a64_fiq:  b unhandled_exception_loop
unhandled_exception_lower_a64_serror: b unhandled_exception_loop
unhandled_exception_lower_a32_sync: b unhandled_exception_loop
unhandled_exception_lower_a32_irq:  b unhandled_exception_loop
unhandled_exception_lower_a32_fiq:  b unhandled_exception_loop
unhandled_exception_lower_a32_serror: b unhandled_exception_loop

// Void HAL CPU Contect Switch Implementation
.global cpu_context_switch_impl
cpu_context_switch_impl:
    ret