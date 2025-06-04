/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/* linker_arm64.ld: Linker script for miniOS v1.7 on ARM64 QEMU virt platform */

OUTPUT_FORMAT("elf64-littleaarch64")
OUTPUT_ARCH(aarch64)
ENTRY(_start)

MEMORY {
    RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 128M
}

NUM_CORES = 4;
CORE_STARTUP_STACK_SIZE = 0x4000; /* 16KB per core for startup */

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
        KEEP(*(.text.boot)) /* Ensure _start is first */
        *(.text .text.*)
        *(.vectors)
    } > RAM : text_ro

    .rodata : ALIGN(4K)
    {
        *(.rodata .rodata.*)
    } > RAM : text_ro

    /* Global Constructors */
	.init_array : ALIGN(8) {
		PROVIDE(__init_array_start = .);
		KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*)))
		KEEP (*(.init_array))
		PROVIDE(__init_array_end = .);
	} >RAM :text_ro

    /* Global Destructors (though likely not used in a kernel without exit) */
    .fini_array : ALIGN(8)
    {
        PROVIDE(__fini_array_start = .);
        KEEP(*(SORT_BY_INIT_PRIORITY(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE(__fini_array_end = .);
    } > RAM : text_ro

    /* Writable initialized data */
    .data : ALIGN(4K)
    {
        *(.data .data.*)
    } > RAM : data_rw

    /* Uninitialized data (zeroed at runtime by _start) */
	.bss (NOLOAD) : ALIGN(4K) { /* NOLOAD means it doesn't take space in the ELF file */
		PROVIDE(_bss_start = .);
		*(.bss .bss.*)
		*(COMMON)
		. = ALIGN(16); /* Ensure BSS end is well-aligned */
		PROVIDE(_bss_end = .);
	} >RAM :data_rw /* Mapped to RAM, RW permissions */

    /* Per-core startup stacks (NOLOAD, addresses used by _start) */
	.startup_stacks (NOLOAD) : ALIGN(16) {
		PROVIDE(_stacks_start = .);
		. = . + (NUM_CORES * CORE_STARTUP_STACK_SIZE);
		PROVIDE(_stacks_end = .); /* Define _stacks_end for completeness */
	} >RAM :data_rw /* Mapped to RAM, RW permissions */

    /* End of the image marker */
    PROVIDE(_end = ALIGN(4K)); /* Aligned end symbol */
    PROVIDE(end = _end);       /* Common alias */

    /DISCARD/ : {
        *(.comment)
        *(.note.*)
        *(.eh_frame)
        *(.ARM.exidx*)      /* Exception handling unwind tables (ARM specific) */
        *(.ARM.attributes)  /* ARM specific build attributes */
        *(.interp)          /* Program interpreter (for dynamic linking, not used) */
        *(.gnu.version*)    /* Versioning sections */
        *(.dynsym)          /* Dynamic symbol table */
        *(.dynstr)          /* Dynamic string table */
        *(.gnu.hash)        /* GNU hash table for dynamic linking */
        *(.rela.*)          /* Relocation sections for dynamic linking */
        *(.dynamic)         /* Dynamic section */
        *(.got)             /* Global Offset Table */
        *(.plt)             /* Procedure Linkage Table */
    }
}