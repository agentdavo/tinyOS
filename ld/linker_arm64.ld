/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/* Linker Script for miniOS ARM64 QEMU virt */

ENTRY(_start)

MEMORY
{
    RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 128M
}

PHDRS
{
    kernel_load PT_LOAD FLAGS(7); /* Read, Write, Execute (RWX) */
}

STACK_SIZE = 0x4000;
NUM_CORES = 4;

SECTIONS
{
    . = ORIGIN(RAM);
    _kernel_start_vma = .;

    .rela.dyn : ALIGN(8)
    {
        *(.rela.dyn)
        *(.rela.plt)
    } > RAM :kernel_load

    .text : ALIGN(4K)
    {
        _text_output_section_start_vma = .;
        KEEP(*(.vectors))
        KEEP(*(.text.boot))
        *(.text .text.*)
        _text_output_section_end_vma = .;
    } > RAM :kernel_load

    .rodata : ALIGN(8)
    {
        *(.rodata .rodata.*)
        . = ALIGN(8);
        __init_array_start = .;
        KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array))
        __init_array_end = .;
        __fini_array_start = .;
        KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array))
        __fini_array_end = .;
        *(.eh_frame_hdr)
        *(.eh_frame)
        *(.plt) *(.plt.*)
        *(.gnu.version) *(.gnu.version_r)
    } > RAM AT> RAM FLAGS(4) :kernel_load /* Read-only (R) */

    . = ALIGN(4K);
    _data_segment_vma_start = .;

    .data : ALIGN(8)
    {
        *(.data .data.*)
        *(.got) *(.got.plt)
        *(.dynamic)
        _data_section_end_vma = .;
    } > RAM :kernel_load

    . = ALIGN(4K);
    .bss (NOLOAD) : ALIGN(8)
    {
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        _bss_end = .;
    } > RAM :kernel_load
    _bss_size = _bss_end - _bss_start;

    . = ALIGN(16);
    .stacks (NOLOAD) :
    {
        _stacks_start = .;
        . = . + (STACK_SIZE * NUM_CORES);
        _stacks_end = .;
    } > RAM :kernel_load

    _kernel_image_end_vma = _stacks_end;

    /DISCARD/ :
    {
        *(.comment)
        *(.note.gnu.build-id)
        *(.ARM.attributes)
        *(.interp)
        *(.dynsym)
        *(.dynstr)
        *(.gnu.hash)
        *(.init)
        *(.fini)
    }
}