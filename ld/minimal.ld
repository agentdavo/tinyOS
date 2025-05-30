SECTIONS {
    . = ORIGIN(RAM);

    .text : {
        KEEP(*(.text.boot))
        *(.text)
    } > RAM

    .data : {
        *(.data)
    } > RAM

    .bss : {
        _bss_start = .;
        *(.bss)
        *(COMMON)
        _bss_end = .;
    } > RAM

    /* Optional stacks section: */
    .stacks : {
        _stacks_start = .;
        *(.stacks)
        _stacks_end = .;
    } > RAM

    /* C++ static constructor array, if used: */
    .init_array : {
        __init_array_start = .;
        KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;
    } > RAM
}
