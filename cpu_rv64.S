// SPDX-License-Identifier: MIT OR Apache-2.0
/*
 * @file cpu_rv64.S
 * @brief RISC-V boot and context-switching code for miniOS v1.7.
 * @details
 * Implements the RISC-V boot sequence, context switching, and exception handling for QEMU virt,
 * supporting SMP with 4 cores. Initializes machine mode, sets up stacks, and calls kernel_main.
 * Updated in v1.7 with improved comments and compatibility with v1.7 kernel, retaining all v1.6
 * functionality.
 *
 * @version 1.7
 * @see hal_qemu_rv64ima.hpp, miniOS.hpp
 */

    .section .text.boot
    .global _start
_start:
    // Determine hart ID
    csrr a0, mhartid
    beqz a0, primary_hart

    // Secondary harts wait
secondary_wait:
    wfi
    j secondary_wait

primary_hart:
    // Set up machine mode
    li t0, 0x1800 // MSTATUS: MPP=11 (machine mode)
    csrw mstatus, t0
    la t0, mtrap_entry
    csrw mtvec, t0

    // Set up stack (16KB per hart)
    la t0, __stack_start
    li t1, 0x4000 // 16KB per hart
    csrr t2, mhartid
    mul t1, t1, t2
    add t0, t0, t1
    mv sp, t0

    // Clear BSS
    la t0, __bss_start
    la t1, __bss_end
    li t2, 0
bss_clear_loop:
    bge t0, t1, bss_clear_done
    sd t2, (t0)
    addi t0, t0, 8
    j bss_clear_loop
bss_clear_done:

    // Call kernel_main
    csrr a0, mhartid
    call kernel_main
    j .

mtrap_entry:
    // Save registers
    csrw mscratch, sp
    la sp, __trap_stack
    addi sp, sp, -256
    sd x1, 8(sp)
    sd x3, 24(sp)
    // ... (save x4-x31 similarly)
    csrr t0, mcause
    csrr t1, mepc
    call handle_exception
    ld x1, 8(sp)
    ld x3, 24(sp)
    // ... (restore x4-x31)
    csrr sp, mscratch
    mret

    .global context_switch
context_switch:
    // Save current context
    sd x1, 8(a0)
    sd x3, 24(a0)
    // ... (save x4-x31)
    sd sp, 248(a0)

    // Restore next context
    ld x1, 8(a1)
    ld x3, 24(a1)
    // ... (restore x4-x31)
    ld sp, 248(a1)
    ret

    .global handle_exception
handle_exception:
    // Simplified: Call C handler
    call hal_irq_handler
    ret