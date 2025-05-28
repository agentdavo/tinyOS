// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file cpu_arm64.S
 * @brief ARM64 CPU specific assembly functions for miniOS v1.7.
 */

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
    and     x0, x0, #0xFF   
    cbz     x0, .L_primary_core_setup 

.L_secondary_core_spin:
    dmb     sy
    wfe
    b       .L_secondary_core_spin

.L_primary_core_setup:
    ldr     x0, =exception_vectors
    msr     vbar_el1, x0
    isb 

    ldr     x1, =_bss_start     
    ldr     x2, =_bss_end
.L_bss_zero_loop:
    cmp     x1, x2
    b.ge    .L_bss_zero_done
    str     xzr, [x1], #8       
    b       .L_bss_zero_loop
.L_bss_zero_done:

    ldr     x1, =_stack_top     
    mov     sp, x1              

    bl      kernel_main         

.L_kernel_halt: 
    wfi                     
    b       .L_kernel_halt

    .section .text
    .align 2

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
    wfi; b unhandled_exception_loop
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
    ret 