/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/*
 * @file linker_rv64.ld
 * @brief RISC-V linker script for miniOS v1.7.
 * @details
 * Defines the memory layout for the RISC-V QEMU virt platform, with 128MB RAM, placing text, data,
 * BSS, stack, and heap sections. Supports 4-core SMP with per-core stacks. Updated in v1.7 with
 * clearer comments and alignment with v1.7 kernel, retaining all v1.6 functionality.
 *
 * @version 1.7
 * @see cpu_rv64.S, miniOS.hpp
 */

MEMORY
{
    RAM (rwx) : ORIGIN = 0x80000000, LENGTH = 128M
}

SECTIONS
{
    . = 0x80000000;

    .text :
    {
        KEEP(*(.text.boot))
        *(.text .text.*)
    } > RAM

    .rodata :
    {
        *(.rodata .rodata.*)
    } > RAM

    .data :
    {
        *(.data .data.*)
    } > RAM

    .bss :
    {
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_end = .;
    } > RAM

    .stack :
    {
        __stack_start = .;
        . = . + 0x10000; /* 64KB for 4 cores (16KB each) */
        __stack_end = .;
    } > RAM

    .heap :
    {
        __heap_start = .;
        . = . + 0x100000; /* 1MB heap */
        __heap_end = .;
    } > RAM

    .trap_stack :
    {
        __trap_stack = .;
        . = . + 0x1000; /* 4KB trap stack */
        __trap_stack_end = .;
    } > RAM

    /DISCARD/ :
    {
        *(.comment)
        *(.eh_frame)
    }
}