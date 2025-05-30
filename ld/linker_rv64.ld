/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/* linker_arm64.ld: Linker script for miniOS v1.7 on ARM64 QEMU virt platform */

OUTPUT_FORMAT("elf64-littleaarch64")
OUTPUT_ARCH(aarch64)
ENTRY(_start)

MEMORY {
    RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 128M
}

NUM_CORES = 4;
CORE_STARTUP_STACK_SIZE = 0x4000;

PHDRS {
    text_ro PT_LOAD FLAGS(5); /* R-X */
    data_rw PT_LOAD FLAGS(6); /* RW- */
}

SECTIONS
{
    . = ORIGIN(RAM);

    /* Code + read-only data */
    .text : ALIGN(4K)
    {
        KEEP(*(.text.boot))
        *(.text .text.*)
        *(.vectors)
    } > RAM : text_ro

    .rodata : ALIGN(4K)
    {
        *(.rodata .rodata.*)
    } > RAM : text_ro

    /* Constructors/destructors */
	.init_array : ALIGN(8) {
		__init_array_start = .;
		KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;
	} >RAM :text_ro

    .fini_array : ALIGN(8)
    {
        PROVIDE(__fini_array_start = .);
        KEEP(*(SORT_BY_INIT_PRIORITY(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE(__fini_array_end = .);
    } > RAM : text_ro

    /* Writable data */
    .data : ALIGN(4K)
    {
        *(.data .data.*)
    } > RAM : data_rw

    /* Uninitialized data (zeroed at runtime) */
	.bss (NOLOAD) : ALIGN(4K) {
		_bss_start = .;
		*(.bss .bss.*)
		*(COMMON)
		. = ALIGN(16);
		_bss_end = .;
	} >RAM :data_rw

    /* Per-core startup stacks (NOLOAD, filled by runtime) */
	.startup_stacks (NOLOAD) : ALIGN(16) {
		_stacks_start = .;
		. = . + (NUM_CORES * CORE_STARTUP_STACK_SIZE);
		_stacks_end = .;
	} >RAM :data_rw

    /* End of the image */
    PROVIDE(_end = ALIGN(4K));
    PROVIDE(end = _end);

    /DISCARD/ : {
        *(.comment)
        *(.note.*)
        *(.eh_frame)
        *(.ARM.exidx*)
        *(.ARM.attributes)
        *(.interp)
        *(.gnu.version*)
        *(.dynsym)
        *(.dynstr)
        *(.gnu.hash)
        *(.rela.*)
        *(.dynamic)
        *(.got)
        *(.plt)
    }
}
