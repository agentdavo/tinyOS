ENTRY(_start)
MEMORY
{
    RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 128M
}

STACK_SIZE = 0x4000;   /* 16KB per core */
NUM_CORES = 4;

SECTIONS
{
    . = ORIGIN(RAM);

    _kernel_start = .;

    .text : ALIGN(4K)
    {
        KEEP(*(.vectors))
        KEEP(*(.text.boot))
        *(.text .text.*)
        *(.rodata .rodata.*)
        *(.plt)
        *(.plt.*)

        /* C++ ctors/dtors */
        . = ALIGN(8);
        PROVIDE(__init_array_start = .);
        KEEP(*(.init_array*))
        PROVIDE(__init_array_end = .);

        . = ALIGN(8);
        PROVIDE(__fini_array_start = .);
        KEEP(*(.fini_array*))
        PROVIDE(__fini_array_end = .);

        KEEP(*(.eh_frame))
        KEEP(*(.eh_frame_hdr))
    } > RAM

    _text_end = .;

    .data : ALIGN(4K)
    {
        *(.data .data.*)
    } > RAM

    .dynamic : ALIGN(8) { *(.dynamic) } > RAM
    .got : ALIGN(8) { *(.got) } > RAM
    .got.plt : ALIGN(8) { *(.got.plt) } > RAM

    .bss (NOLOAD) : ALIGN(4K)
    {
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        _bss_end = .;
    } > RAM

    .stacks (NOLOAD) : ALIGN(4K)
    {
        _stacks_start = .;
        . = . + (STACK_SIZE * NUM_CORES);
        _stacks_end = .;
    } > RAM

    _kernel_end = .;

    /DISCARD/ : { *(.comment) *(.note*) *(.ARM.attributes) *(.interp) *(.dynsym) *(.dynstr) *(.gnu.hash) *(.rela.dyn) }
}
