/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/* linker_rv64.ld — miniOS on QEMU virt RISC-V (rv64 + M-mode, -bios none). */

OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY {
    RAM (rwx) : ORIGIN = 0x80000000, LENGTH = 128M
}

NUM_CORES = 4;
CORE_STARTUP_STACK_SIZE = 0x4000; /* 16KB per hart */

PHDRS {
    text_ro PT_LOAD FLAGS(5); /* R-X */
    data_rw PT_LOAD FLAGS(6); /* RW- */
}

SECTIONS {
    . = ORIGIN(RAM);

    .text : ALIGN(4K) {
        KEEP(*(.text.boot))
        *(.text .text.*)
    } > RAM : text_ro

    .rodata : ALIGN(4K) {
        *(.rodata .rodata.*)
    } > RAM : text_ro

    .init_array : ALIGN(8) {
        PROVIDE(__init_array_start = .);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end = .);
    } > RAM : text_ro

    .data : ALIGN(4K) {
        *(.data .data.*)
    } > RAM : data_rw

    .bss (NOLOAD) : ALIGN(4K) {
        PROVIDE(_bss_start = .);
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(16);
        PROVIDE(_bss_end = .);
    } > RAM : data_rw

    .stacks (NOLOAD) : ALIGN(16) {
        PROVIDE(_stacks_start = .);
        . = . + (NUM_CORES * CORE_STARTUP_STACK_SIZE);
        PROVIDE(_stacks_end = .);
    } > RAM : data_rw

    PROVIDE(_end = ALIGN(4K));

    /DISCARD/ : {
        *(.comment)
        *(.note.*)
        *(.eh_frame)
        *(.riscv.attributes)
    }
}
