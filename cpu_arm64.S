/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/**
 * @file cpu_arm64.S
 * @brief ARM64 CPU specific assembly functions for miniOS v1.7.
 */

    .equ STACK_SIZE_ASM, 0x4000   
    .equ UART_BASE,    0x09000000
    .equ UARTFR,       0x18
    .equ UARTDR,       0x00
    .equ UART_TXFF,    (1 << 5)  

    .equ TCB_REGS_X0_OFFSET, (0*8)
    .equ TCB_REGS_X30_OFFSET, (30*8)
    .equ TCB_SP_OFFSET, (31*8)      
    .equ TCB_PC_OFFSET, (31*8 + 8)  
    .equ TCB_PSTATE_OFFSET, (31*8 + 16) 

.extern __init_array_start
.extern __init_array_end
.extern _bss_start
.extern _bss_end
.extern _stacks_start
.extern _stacks_end

    .section .rodata
.L_debug_start:     .asciz "[DEBUG] _start ENTRY\n"
.L_debug_stack:     .asciz "[DEBUG] Stack set up\n"
.L_debug_primary:   .asciz "[DEBUG] Primary core setup\n"
.L_debug_vectors:   .asciz "[DEBUG] Exception vectors set\n"
.L_debug_constructors: .asciz "[DEBUG] Calling constructors\n"
.L_debug_bss:       .asciz "[DEBUG] BSS zeroed\n"
.L_debug_kernel_main: .asciz "[DEBUG] Entering kernel_main\n"

    .section .vectors, "ax", %progbits 
    .align 11 
.global exception_vectors 
exception_vectors:
    b       unhandled_exception_sp0_sync
    .align 7 
    b       unhandled_exception_sp0_irq
    .align 7
    b       unhandled_exception_sp0_fiq
    .align 7
    b       unhandled_exception_sp0_serror
    .align 7
    b       unhandled_exception_spx_sync
    .align 7
    b       current_el_spx_irq_entry
    .align 7 
    b       unhandled_exception_spx_fiq
    .align 7
    b       unhandled_exception_spx_serror
    .align 7
    b       unhandled_exception_lower_a64_sync
    .align 7
    b       unhandled_exception_lower_a64_irq
    .align 7
    b       unhandled_exception_lower_a64_fiq
    .align 7
    b       unhandled_exception_lower_a64_serror
    .align 7
    b       unhandled_exception_lower_a32_sync
    .align 7
    b       unhandled_exception_lower_a32_irq
    .align 7
    b       unhandled_exception_lower_a32_fiq
    .align 7
    b       unhandled_exception_lower_a32_serror
    .align 7

    .section .text.boot, "ax", %progbits
    .global _start  
_start:
    /* Print "[DEBUG] _start ENTRY" */
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0
    adr     x0, .L_debug_start
    bl      early_uart_puts
    mov     x0, x19
    ldp     x0, x1, [sp], #16

    mrs     x0, mpidr_el1
    and     x0, x0, #0xFF               // x0 = current core_id

    ldr     x1, =_stacks_start          
    mov     x2, #STACK_SIZE_ASM         
    mul     x3, x0, x2                  // x3 = core_id * STACK_SIZE_ASM
    add     x1, x1, x3                  // x1 = _stacks_start + offset
    add     x1, x1, x2                  // x1 = stack top for this core
    mov     sp, x1                      // Set stack pointer

    /* Print "[DEBUG] Stack set up" */
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0
    adr     x0, .L_debug_stack
    bl      early_uart_puts
    mov     x0, x19
    ldp     x0, x1, [sp], #16

    cbz     x0, .L_primary_core_continue_setup 

.L_secondary_core_spin:
    dmb     sy
    wfe
    b       .L_secondary_core_spin

.L_primary_core_continue_setup:
    /* Print "[DEBUG] Primary core setup" */
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0
    adr     x0, .L_debug_primary
    bl      early_uart_puts
    mov     x0, x19
    ldp     x0, x1, [sp], #16

    ldr     x0, =exception_vectors
    msr     vbar_el1, x0
    isb 

    /* Print "[DEBUG] Exception vectors set" */
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0
    adr     x0, .L_debug_vectors
    bl      early_uart_puts
    mov     x0, x19
    ldp     x0, x1, [sp], #16

    // ENABLE NEON/SIMD ACCESS for EL1 BEFORE C++ CONSTRUCTORS
    mrs     x0, cpacr_el1
    orr     x0, x0, #(3 << 20)    // Set FPEN to 0b11
    msr     cpacr_el1, x0
    isb                           // Synchronize context changes

    /* Print "[DEBUG] Calling constructors" */
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0
    adr     x0, .L_debug_constructors
    bl      early_uart_puts
    mov     x0, x19
    ldp     x0, x1, [sp], #16

    bl      call_constructors 

    /* Print "[DEBUG] BSS zeroed" */
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0
    adr     x0, .L_debug_bss
    bl      early_uart_puts
    mov     x0, x19
    ldp     x0, x1, [sp], #16

    ldr     x1, =_bss_start
    ldr     x2, =_bss_end
.L_bss_zero_loop:
    cmp     x1, x2
    b.ge    .L_bss_zero_done
    str     xzr, [x1], #8
    b       .L_bss_zero_loop
.L_bss_zero_done:

    /* Print "[DEBUG] Entering kernel_main" */
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0
    adr     x0, .L_debug_kernel_main
    bl      early_uart_puts
    mov     x0, x19
    ldp     x0, x1, [sp], #16

    bl      kernel_main

    // After kernel_main, load initial thread context for core 0
    ldr     x0, =kernel_g_per_cpu_data  // Address of kernel::core::g_per_cpu_data
    ldr     x1, [x0, #0]                // Load g_per_cpu_data[0].current_thread (TCB*)
    cbz     x1, .L_kernel_halt          // If null, halt

    // Restore thread context from TCB
    add     x2, x1, #TCB_SP_OFFSET      // Offset to sp in TCB
    ldr     x3, [x2]                    // Load sp
    mov     sp, x3                      // Set stack pointer
    add     x2, x2, #8                  // Offset to pc
    ldr     x3, [x2]                    // Load pc
    add     x2, x2, #8                  // Offset to pstate
    ldr     x4, [x2]                    // Load pstate
    msr     spsr_el1, x4                // Set SPSR_EL1 (pstate)
    mov     x30, x3                     // Set LR to pc (eret will use this)
    // Restore registers x0-x30
    ldp     x0, x1, [x1, #TCB_REGS_X0_OFFSET]
    ldp     x2, x3, [x1, #TCB_REGS_X0_OFFSET + 16]
    ldp     x4, x5, [x1, #TCB_REGS_X0_OFFSET + 32]
    ldp     x6, x7, [x1, #TCB_REGS_X0_OFFSET + 48]
    ldp     x8, x9, [x1, #TCB_REGS_X0_OFFSET + 64]
    ldp     x10, x11, [x1, #TCB_REGS_X0_OFFSET + 80]
    ldp     x12, x13, [x1, #TCB_REGS_X0_OFFSET + 96]
    ldp     x14, x15, [x1, #TCB_REGS_X0_OFFSET + 112]
    ldp     x16, x17, [x1, #TCB_REGS_X0_OFFSET + 128]
    ldp     x18, x19, [x1, #TCB_REGS_X0_OFFSET + 144]
    ldp     x20, x21, [x1, #TCB_REGS_X0_OFFSET + 160]
    ldp     x22, x23, [x1, #TCB_REGS_X0_OFFSET + 176]
    ldp     x24, x25, [x1, #TCB_REGS_X0_OFFSET + 192]
    ldp     x26, x27, [x1, #TCB_REGS_X0_OFFSET + 208]
    ldp     x28, x29, [x1, #TCB_REGS_X0_OFFSET + 224]
    ldr     x30, [x1, #TCB_REGS_X30_OFFSET]  // Restore x30 (LR)
    eret                                // Switch to thread context

.L_kernel_halt:
    wfi
    b       .L_kernel_halt

    .section .text
    .align 2

.global call_constructors
call_constructors:
    mov     x19, lr         
    ldr     x0, =__init_array_start 
    ldr     x1, =__init_array_end   
.L_init_loop:
    cmp     x0, x1          
    b.ge    .L_init_done    
    ldr     x2, [x0], #8    
    cbz     x2, .L_init_loop 
    blr     x2              
    b       .L_init_loop    
.L_init_done:
    mov     lr, x19         
    ret                     

current_el_spx_irq_entry:
    sub     sp, sp, #176                            
    stp     x0, x1, [sp, #(0*16)]                   
    stp     x2, x3, [sp, #(1*16)]                   
    stp     x4, x5, [sp, #(2*16)]                   
    stp     x6, x7, [sp, #(3*16)]
    stp     x8, x9, [sp, #(4*16)]
    stp     x10, x11, [sp, #(5*16)]
    stp     x12, x13, [sp, #(6*16)]
    stp     x14, x15, [sp, #(7*16)]
    stp     x16, x17, [sp, #(8*16)]                 
    mrs     x0, elr_el1                             
    mrs     x1, spsr_el1                            
    mov     x2, lr                                  
    stp     x0, x1, [sp, #(9*16)]                   
    str     x2, [sp, #(9*16 + 16)]                  
    mrs     x0, mpidr_el1
    and     x0, x0, #0xFF                           
    bl      hal_irq_handler                         
    ldr     x2, [sp, #(9*16 + 16)]                  
    mov     lr, x2                                  
    ldp     x0, x1, [sp, #(9*16)]                   
    msr     elr_el1, x0                             
    msr     spsr_el1, x1                            
    ldp     x0, x1, [sp, #(0*16)]                   
    ldp     x2, x3, [sp, #(1*16)]                   
    ldp     x4, x5, [sp, #(2*16)]                   
    ldp     x6, x7, [sp, #(3*16)]
    ldp     x8, x9, [sp, #(4*16)]
    ldp     x10, x11, [sp, #(5*16)]
    ldp     x12, x13, [sp, #(6*16)]
    ldp     x14, x15, [sp, #(7*16)]
    ldp     x16, x17, [sp, #(8*16)]                 
    add     sp, sp, #176                            
    eret                                            

unhandled_exception_loop:
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0
    ldr     x0, =UART_BASE
    mov     w1, #'Z'
.L_uart_Z_wait:  
    ldr     w2, [x0, #UARTFR]
    tst     w2, #UART_TXFF
    b.ne    .L_uart_Z_wait
    str     w1, [x0, #UARTDR]
    mov     x0, x19
    ldp     x19, x1, [sp], #16
.unhandled_exc_loop_wfi:
    wfi
    b       .unhandled_exc_loop_wfi

unhandled_exception_sp0_sync:   b unhandled_exception_loop
unhandled_exception_sp0_irq:    b unhandled_exception_loop
unhandled_exception_sp0_fiq:    b unhandled_exception_loop
unhandled_exception_sp0_serror: b unhandled_exception_loop
unhandled_exception_spx_sync:   b unhandled_exception_loop
unhandled_exception_spx_fiq:    b unhandled_exception_loop
unhandled_exception_spx_serror: b unhandled_exception_loop
unhandled_exception_lower_a64_sync: b unhandled_exception_loop
unhandled_exception_lower_a64_irq:  b unhandled_exception_loop
unhandled_exception_lower_a64_fiq:  b unhandled_exception_loop
unhandled_exception_lower_a64_serror:b unhandled_exception_loop
unhandled_exception_lower_a32_sync: b unhandled_exception_loop
unhandled_exception_lower_a32_irq:  b unhandled_exception_loop
unhandled_exception_lower_a32_fiq:  b unhandled_exception_loop
unhandled_exception_lower_a32_serror:b unhandled_exception_loop

.global cpu_context_switch_impl
cpu_context_switch_impl:
    // x0 = old_tcb, x1 = new_tcb
    // If old_tcb is null (x0 == 0), skip saving context
    cbz     x0, .L_restore_new_context

    // Save old TCB context
    // Save x0-x30
    stp     x0, x1, [x0, #TCB_REGS_X0_OFFSET]
    stp     x2, x3, [x0, #TCB_REGS_X0_OFFSET + 16]
    stp     x4, x5, [x0, #TCB_REGS_X0_OFFSET + 32]
    stp     x6, x7, [x0, #TCB_REGS_X0_OFFSET + 48]
    stp     x8, x9, [x0, #TCB_REGS_X0_OFFSET + 64]
    stp     x10, x11, [x0, #TCB_REGS_X0_OFFSET + 80]
    stp     x12, x13, [x0, #TCB_REGS_X0_OFFSET + 96]
    stp     x14, x15, [x0, #TCB_REGS_X0_OFFSET + 112]
    stp     x16, x17, [x0, #TCB_REGS_X0_OFFSET + 128]
    stp     x18, x19, [x0, #TCB_REGS_X0_OFFSET + 144]
    stp     x20, x21, [x0, #TCB_REGS_X0_OFFSET + 160]
    stp     x22, x23, [x0, #TCB_REGS_X0_OFFSET + 176]
    stp     x24, x25, [x0, #TCB_REGS_X0_OFFSET + 192]
    stp     x26, x27, [x0, #TCB_REGS_X0_OFFSET + 208]
    stp     x28, x29, [x0, #TCB_REGS_X0_OFFSET + 224]
    str     x30, [x0, #TCB_REGS_X30_OFFSET]
    // Save SP
    mov     x2, sp
    str     x2, [x0, #TCB_SP_OFFSET]
    // Save PC (use caller’s LR as PC)
    str     x30, [x0, #TCB_PC_OFFSET]
    // Save PSTATE (SPSR_EL1)
    mrs     x2, spsr_el1
    str     x2, [x0, #TCB_PSTATE_OFFSET]

.L_restore_new_context:
    // Restore new TCB context
    // Load SP
    ldr     x2, [x1, #TCB_SP_OFFSET]
    mov     sp, x2
    // Load PC into x30
    ldr     x3, [x1, #TCB_PC_OFFSET]
    // Load PSTATE
    ldr     x4, [x1, #TCB_PSTATE_OFFSET]
    msr     spsr_el1, x4
    // Load x0-x30
    ldp     x0, x2, [x1, #TCB_REGS_X0_OFFSET]  // Load x0, x2 (x1 will be loaded later)
    ldp     x4, x5, [x1, #TCB_REGS_X0_OFFSET + 16]
    ldp     x6, x7, [x1, #TCB_REGS_X0_OFFSET + 32]
    ldp     x8, x9, [x1, #TCB_REGS_X0_OFFSET + 48]
    ldp     x10, x11, [x1, #TCB_REGS_X0_OFFSET + 64]
    ldp     x12, x13, [x1, #TCB_REGS_X0_OFFSET + 80]
    ldp     x14, x15, [x1, #TCB_REGS_X0_OFFSET + 96]
    ldp     x16, x17, [x1, #TCB_REGS_X0_OFFSET + 112]
    ldp     x18, x19, [x1, #TCB_REGS_X0_OFFSET + 128]
    ldp     x20, x21, [x1, #TCB_REGS_X0_OFFSET + 144]
    ldp     x22, x23, [x1, #TCB_REGS_X0_OFFSET + 160]
    ldp     x24, x25, [x1, #TCB_REGS_X0_OFFSET + 176]
    ldp     x26, x27, [x1, #TCB_REGS_X0_OFFSET + 192]
    ldp     x28, x29, [x1, #TCB_REGS_X0_OFFSET + 208]
    ldp     x1, x30, [x1, #TCB_REGS_X0_OFFSET + 224]  // Load x1, x30
    mov     x30, x3  // Set LR to PC
    eret