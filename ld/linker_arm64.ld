MEMORY
{
    RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 128M
}

SECTIONS
{
    . = 0x40000000;
    .text : { *(.text) } > RAM
    .data : { *(.data) } > RAM
    .bss : { *(.bss) } > RAM
}
