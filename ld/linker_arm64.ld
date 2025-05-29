/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/* Linker Script for miniOS ARM64 QEMU virt - Final Segment Fix */

ENTRY(_start) 

MEMORY
{
    RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 128M 
}

PHDRS
{
    text_segment PT_LOAD FLAGS(5); /* Read, Execute */
    data_segment PT_LOAD FLAGS(6); /* Read, Write */
}

/* Define symbols for stack configuration, matching assembly usage */
STACK_SIZE = 0x4000;
NUM_CORES = 4;

SECTIONS
{
    /* === Segment 1: Code and Read-Only Data (text_segment PHDR) === */
    . = ORIGIN(RAM); /* VMA for this segment starts at 0x40000000 */
    _kernel_start_vma = .;
    
    /* The .text output section will collect all input sections assigned to it.
       It defines the content of the 'text_segment' program header. */
    .text : ALIGN(4K) 
    {
        _text_output_section_start_vma = .;
        KEEP(*(.vectors))    
        KEEP(*(.text.boot))  
        *(.text .text.*)     
        
        . = ALIGN(8);
        *(.rodata .rodata.*) 
        
        . = ALIGN(8);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) .init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);

        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) .fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);

        *(.glue_7)           
        *(.glue_7t)
        KEEP(*(.eh_frame_hdr)) /* eh_frame_hdr usually comes before eh_frame */
        KEEP(*(.eh_frame))     
        
        . = ALIGN(8);
        *(.plt) *(.plt.*) 
        
        . = ALIGN(8);
        *(.gnu.version) 
        *(.gnu.version_r)
        *(.rela.dyn)    
        *(.rela.plt)

        . = ALIGN(8); 
        _text_output_section_end_vma = .; /* End of VMA for all content in this output section */
    } > RAM :text_segment 
    _text_segment_lma_end = LOADADDR(.text) + SIZEOF(.text); /* End LMA of text_segment in ELF file */


    /* === Segment 2: Read-Write Data (data_segment PHDR) === */
    /* VMA for this segment must start aligned AFTER the previous segment's VMA extent. */
    /* LMA for this segment must start AFTER the previous segment's LMA extent in the file. */
    
    . = _text_output_section_end_vma; /* VMA continues from where the .text output section ended in MEMORY */
    . = ALIGN(4K);                    /* Align this VMA for the start of the data segment */
    _data_segment_vma_start = .;      /* This VMA is used for the data_segment PHDR */

    /* The .data output section. Its VMA starts at _data_segment_vma_start. */
    /* Its LMA starts after the file portion of the text_segment. */
    .data : AT (_text_segment_lma_end) 
    {
        /* _data_section_start_vma = .; // This would be same as _data_segment_vma_start */
        *(.data .data.*)    
        
        . = ALIGN(8);
        *(.got)       
        *(.got.plt)   
        
        . = ALIGN(8);
        *(.dynamic) /* If present */

        . = ALIGN(8);
        _data_section_end_vma = .; /* End of VMA for .data and explicitly placed RW sections */
    } > RAM :data_segment 
    /* _data_segment_size_in_file = SIZEOF(.data); // Size of .data output section in file */


    /* BSS section. VMA follows .data sections. NOLOAD. Part of data_segment PHDR for memory. */
    /* Its VMA will be current '.', which is _data_section_end_vma, aligned up. */
    . = _data_section_end_vma; /* Ensure BSS VMA follows data VMA */
    . = ALIGN(4K); /* Align VMA for .bss if needed after .data block */
    .bss (NOLOAD) : 
    {
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(8);
        _bss_end = .;
    } > RAM :data_segment /* Also assign to data_segment PHDR */
    _bss_size = _bss_end - _bss_start; 


    /* Stack section. VMA follows BSS. NOLOAD. */
    . = _bss_end; /* Start stack VMA after BSS */
    . = ALIGN(16); 
    .stacks (NOLOAD) : 
    {
        _stacks_start = .; 
        . = . + (STACK_SIZE * NUM_CORES);       
        _stacks_end = .;    
    } > RAM /* Not part of a PT_LOAD segment for file loading purposes */
    
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
        /* .rela.dyn and .rela.plt are attempted to be placed in .text segment */
        /* If they are not found by those wildcards, they will be discarded if not placed elsewhere */
    }
}