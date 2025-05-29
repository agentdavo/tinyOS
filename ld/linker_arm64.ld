/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/* Linker Script for miniOS ARM64 QEMU virt - Strict Initial Section Order */

ENTRY(_start) 

MEMORY
{
    RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 128M 
}

PHDRS
{
    kernel_code PT_LOAD FLAGS(5); /* Read, Execute for all code/RO data */
    kernel_data PT_LOAD FLAGS(6); /* Read, Write for all data/BSS */
}

STACK_SIZE = 0x4000;   
NUM_CORES = 4;         

SECTIONS
{
    /* === Segment 1: Code and Read-Only Data (kernel_code PHDR) === */
    . = ORIGIN(RAM); 
    _kernel_start_vma = .;

    /* Critical boot sections first */
    .vectors : ALIGN(4K) /* Vector table must be aligned for VBAR */
    {
        KEEP(*(.vectors))    
    } > RAM :kernel_code

    .text.boot : ALIGN(4K) /* _start and early setup */
    {
        KEEP(*(.text.boot))  
    } > RAM :kernel_code

    /* Other code and read-only data */
    .text_others : ALIGN(4K)
    {
        *(.text .text.*)     /* All other function code */
        
        . = ALIGN(8);
        *(.rodata .rodata.*) /* Read-only data */
        
        . = ALIGN(8);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*crtbegin.o(.init_array)) 
        KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) .init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);

        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*crtbegin.o(.fini_array)) 
        KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) .fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
        
        KEEP(*(.eh_frame_hdr)) 
        KEEP(*(.eh_frame))     
        
        . = ALIGN(8);
        *(.plt) *(.plt.*) 
        
        /* Place other RO dynamic sections explicitly after our main code/rodata */
        . = ALIGN(8);
        *(.gnu.version) 
        *(.gnu.version_r)
        *(.rela.dyn)    /* These often contain RO data for relocations */
        *(.rela.plt)

        . = ALIGN(8); 
    } > RAM :kernel_code
    _text_segment_end_lma = LOADADDR(.text_others) + SIZEOF(.text_others); /* LMA of where this segment ends */
                                                                    /* This uses .text_others as it's the last named output section in this PHDR */


    /* === Segment 2: Read-Write Data (kernel_data PHDR) === */
    . = ALIGN(4K); /* Align VMA for the start of the data segment block */
    _data_segment_start_vma = .;

    /* The .data output section. Its VMA starts at _data_segment_start_vMA. */
    /* Its LMA starts after the file portion of the kernel_code PHDR. */
    .data : AT (_text_segment_end_lma) 
    {
        *(.data .data.*)    
        
        . = ALIGN(8);
        *(.got)       
        *(.got.plt)   
        
        . = ALIGN(8);
        *(.dynamic) /* If present and RW */

        . = ALIGN(8);
    } > RAM :kernel_data 
    _data_segment_end_lma = LOADADDR(.data) + SIZEOF(.data);
    _data_output_section_end_vma = ADDR(.data) + SIZEOF(.data);


    /* BSS section. VMA follows .data sections. NOLOAD. Part of kernel_data PHDR. */
    . = _data_output_section_end_vma; 
    . = ALIGN(4K); 
    .bss (NOLOAD) : 
    {
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(8);
        _bss_end = .;
    } > RAM :kernel_data 
    _bss_size = _bss_end - _bss_start; 

    /* Stack section. VMA follows BSS. NOLOAD. */
    . = _bss_end; 
    . = ALIGN(16); 
    .stacks (NOLOAD) : 
    {
        _stacks_start = .; 
        . = . + (STACK_SIZE * NUM_CORES);       
        _stacks_end = .;    
    } > RAM 
    
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
        /* Sections explicitly placed should not be here. */
        /* If any "could not find section" error appears for .gnu.version*, remove them from .text_others */
    }
}