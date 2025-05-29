// SPDX-License-Identifier: MIT OR Apache-2.0
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
    mrs     x0, mpidr_el1
    and     x0, x0, #0xFF               // x0 = current core_id

    ldr     x1, =_stacks_start          
    mov     x2, #STACK_SIZE_ASM         
    mul     x3, x0, x2                  
    add     x1, x1, x3                  
    add     x1, x1, x2                  
    mov     sp, x1                      

    // DEBUG: Output 'A' after setting stack
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0         // Save original x0 (core_id)
    ldr     x0, =UART_BASE
    mov     w1, #'A'
.L_uart_A_wait:  ldr w2, [x0, #UARTFR]; tst w2, #UART_TXFF; b.ne .L_uart_A_wait
    str     w1, [x0, #UARTDR]
    mov     x0, x19         // Restore original x0
    ldp     x19, x1, [sp], #16 // Restore x1 (x19 was just used as temp for original x0)

    cbz     x0, .L_primary_core_continue_setup 

.L_secondary_core_spin:
    dmb     sy
    wfe
    b       .L_secondary_core_spin

.L_primary_core_continue_setup:
    // DEBUG: Output 'B' after reaching primary core setup
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0
    ldr     x0, =UART_BASE
    mov     w1, #'B'
.L_uart_B_wait:  ldr w2, [x0, #UARTFR]; tst w2, #UART_TXFF; b.ne .L_uart_B_wait
    str     w1, [x0, #UARTDR]
    mov     x0, x19
    ldp     x19, x1, [sp], #16

    ldr     x0, =exception_vectors
    msr     vbar_el1, x0
    isb 

    // *** ENABLE NEON/SIMD ACCESS for EL1 BEFORE C++ CONSTRUCTORS ***
    // CPACR_EL1.FPEN (bits [21:20]):
    // 00: Trap EL0 and EL1 accesses to EL1 (or EL2 if active)
    // 01: Trap EL0 accesses to EL1. EL1 accesses enabled.
    // 10: Trap EL0 and EL1 accesses to EL1 (or EL2 if active)
    // 11: Enable EL0 and EL1 access (no trap).
    // We want 0b11 if tasks might run in EL0, or 0b01 if only EL1 uses NEON.
    // For kernel static initializers running in EL1, 0b01 is sufficient.
    // For general safety and if EL0 tasks might use NEON later, 0b11 is better.
    mrs     x0, cpacr_el1
    orr     x0, x0, #(3 << 20)    // Set FPEN to 0b11
    msr     cpacr_el1, x0
    isb                           // Synchronize context changes
    // ESB is an older instruction synchronization barrier, ISB is preferred for AArch64.
    // After msr to system register, an ISB is good practice.

    // DEBUG: Output 'C' before calling constructors
    stp     x0, x1, [sp, #-16]! // x0 is clobbered by UART, x1 is scratch
    mov     x19, x0             // Save current x0 if needed after UART
    ldr     x0, =UART_BASE
    mov     w1, #'C'
.L_uart_C_wait:  ldr w2, [x0, #UARTFR]; tst w2, #UART_TXFF; b.ne .L_uart_C_wait
    str     w1, [x0, #UARTDR]
    mov     x0, x19             // Restore original x0 (if it was important)
    ldp     x19, x1, [sp], #16

    bl      call_constructors 

    // DEBUG: Output 'E' before BSS zeroing
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0
    ldr     x0, =UART_BASE
    mov     w1, #'E'
.L_uart_E_wait:  ldr w2, [x0, #UARTFR]; tst w2, #UART_TXFF; b.ne .L_uart_E_wait
    str     w1, [x0, #UARTDR]
    mov     x0, x19
    ldp     x19, x1, [sp], #16

    ldr     x1, =_bss_start
    ldr     x2, =_bss_end
.L_bss_zero_loop:
    cmp     x1, x2
    b.ge    .L_bss_zero_done
    str     xzr, [x1], #8
    b       .L_bss_zero_loop
.L_bss_zero_done:

    // DEBUG: Output 'F' before kernel_main
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0
    ldr     x0, =UART_BASE
    mov     w1, #'F'
.L_uart_F_wait:  ldr w2, [x0, #UARTFR]; tst w2, #UART_TXFF; b.ne .L_uart_F_wait
    str     w1, [x0, #UARTDR]
    mov     x0, x19
    ldp     x19, x1, [sp], #16

    bl      kernel_main // Line 142

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
.L_init_loop: // Line 156
    cmp     x0, x1          
    b.ge    .L_init_done    
    ldr     x2, [x0], #8    
    cbz     x2, .L_init_loop 
    blr     x2              
    b       .L_init_loop    // Line 162
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

unhandled_exception_loop: // Line 139 in your GDB 'list' output
    stp     x0, x1, [sp, #-16]!
    mov     x19, x0
    ldr     x0, =UART_BASE
    mov     w1, #'Z'
.L_uart_Z_wait:  ldr w2, [x0, #UARTFR]; tst w2, #UART_TXFF; b.ne .L_uart_Z_wait
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
unhandled_exception_spx_sync:   b unhandled_exception_loop // Line 147 in GDB
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
    ret