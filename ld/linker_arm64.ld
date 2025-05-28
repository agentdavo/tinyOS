/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/* Linker Script for miniOS ARM64 QEMU virt */

ENTRY(_start) /* Define the entry point of the kernel */

MEMORY
{
    /* QEMU virt machine for AArch64 typically provides RAM starting at 0x40000000 */
    ram (rwx) : ORIGIN = 0x40000000, LENGTH = 128M 
}

SECTIONS
{
    /* Kernel image is loaded at 0x40000000 by QEMU's -kernel option */
    . = 0x40000000; 

    .text : ALIGN(4K) /* Code and Read-Only Data sections */
    {
        KEEP(*(.vectors))    /* Exception Vector Table first. Must be correctly aligned by VBAR_EL1 setting. */
        KEEP(*(.text.boot))  /* Boot text section (contains _start) next */
        *(.text .text.*)
        *(.rodata .rodata.*)
        *(.glue_7)           /* ARM/Thumb interworking glue (legacy, but good practice) */
        *(.glue_7t)
        . = ALIGN(8); 
        _text_end = .;
    } > ram :text 

    .data : ALIGN(4K) /* Initialized Read-Write Data */
    {
        _data_start = .;
        *(.data .data.*)
        . = ALIGN(8);
        _data_end = .;
    } > ram :data 

    .bss : ALIGN(4K) /* Uninitialized Read-Write Data (zeroed at boot by _start) */
    {
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(8);
        _bss_end = .;
    } > ram :bss 
    _bss_size = _bss_end - _bss_start; /* Symbol for _start to know BSS size */

    .stack (NOLOAD) : ALIGN(16) /* Initial kernel stack for EL1 */
    {
        _stack_bottom = .;
        . = . + 16K; /* 16KB initial kernel stack. Adjust size as needed. */
        _stack_top = .; /* Stack grows downwards, SP initialized to this */
    } > ram :stack 

    _kernel_end = .; /* Symbol marking the end of the used kernel image */

    /DISCARD/ :
    {
        *(.comment)
        *(.note.gnu.build-id) 
        *(.ARM.attributes)    
    }
}

PHDRS
{
    text PT_LOAD FLAGS(5); /* Read, Execute (RX) = 4 (R) + 1 (X) */
    data PT_LOAD FLAGS(6); /* Read, Write (RW) = 4 (R) + 2 (W) */
    bss  PT_LOAD FLAGS(6); /* Read, Write (RW) = 4 (R) + 2 (W) - for allocation, actual data is zeroed */
    stack PT_LOAD FLAGS(6);/* Read, Write (RW) for stack segment if it needs a PHDR */
}