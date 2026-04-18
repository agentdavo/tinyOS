MEMORY {
    RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 128M
}

SECTIONS {
    . = ORIGIN(RAM);

    .text : {
        KEEP(*(.text.boot))
        *(.text .text.*)
        *(.vectors)
    } > RAM

    .rodata : {
        *(.rodata .rodata.*)
    } > RAM

    .data : {
        *(.data .data.*)
    } > RAM

    .bss : {
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        _bss_end = .;
    } > RAM

    .stacks : {
        _stacks_start = .;
        *(.stacks)
        /* boot_stacks reserves the required memory */
        _stacks_end = .;
    } > RAM

    .init_array : {
        __init_array_start = .;
        KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;
    } > RAM

    _end = .;
}
