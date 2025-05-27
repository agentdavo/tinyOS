// SPDX-License-Identifier: MIT OR Apache-2.0
/*
 * @file cpu_arm64.S
 * @brief ARM64 boot and context-switching code for miniOS v1.7.
 * @details
 * Implements the ARM64 boot sequence, context switching, and exception handling for QEMU virt,
 * supporting SMP with 4 cores. Initializes EL1, sets up stacks, and calls kernel_main. Updated
 * in v1.7 with improved comments and compatibility with v1.7 kernel, retaining all v1.6
 * functionality.
 *
 * @version 1.7
 * @see hal_qemu_arm64.hpp, miniOS.hpp
 */

    .section .text.boot
    .global _start
_start:
    // Determine core ID
    mrs x0, mpidr_el1
    and x0, x0, #0xFF
    cbz x0, primary_core

    // Secondary cores wait
secondary_wait:
    wfe
    b secondary_wait

primary_core:
    // Set up EL1
    mov x0, #0x3C5 // EL1h, interrupts disabled
    msr spsr_el2, x0
    adr x0, el1_entry
    msr elr_el2, x0
    eret

el1_entry:
    // Disable MMU
    mrs x0, sctlr_el1
    bic x0, x0, #1
    msr sctlr_el1, x0

    // Set up stack (16KB per core)
    ldr x0, =__stack_start
    mov x1, #0x4000 // 4KB per core
    mrs x2, mpidr_el1
    and x2, x2, #0xFF
    mul x1, x1, x2
    add x0, x0, x1
    mov sp, x0

    // Clear BSS
    ldr x0, =__bss_start
    ldr x1, =__bss_end
    mov x2, #0
bss_clear_loop:
    cmp x0, x1
    bge bss_clear_done
    str x2, [x0], #8
    b bss_clear_loop
bss_clear_done:

    // Call kernel_main
    mrs x0, mpidr_el1
    and x0, x0, #0xFF
    bl kernel_main
    b .

    .global context_switch
context_switch:
    // Save current context
    stp x19, x20, [x0, #16 * 0]
    stp x21, x22, [x0, #16 * 1]
    stp x23, x24, [x0, #16 * 2]
    stp x25, x26, [x0, #16 * 3]
    stp x27, x28, [x0, #16 * 4]
    stp x29, x30, [x0, #16 * 5]
    mov x2, sp
    str x2, [x0, #16 * 6]

    // Restore next context
    ldp x19, x20, [x1, #16 * 0]
    ldp x21, x22, [x1, #16 * 1]
    ldp x23, x24, [x1, #16 * 2]
    ldp x25, x26, [x1, #16 * 3]
    ldp x27, x28, [x1, #16 * 4]
    ldp x29, x30, [x1, #16 * 5]
    ldr x2, [x1, #16 * 6]
    mov sp, x2

    ret

    .section .text
    .global exception_handler
exception_handler:
    // Save registers
    stp x0, x1, [sp, #-16]!
    mrs x0, esr_el1
    mrs x1, elr_el1
    bl handle_exception
    ldp x0, x1, [sp], #16
    eret

    .global handle_exception
handle_exception:
    // Simplified: Call C handler
    b hal_irq_handler