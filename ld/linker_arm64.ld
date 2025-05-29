/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/* Linker Script for miniOS ARM64 QEMU virt - Corrected BSS Alignment */

ENTRY(_start) 

MEMORY
{
    RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 128M 
}

PHDRS
{
    kernel_load PT_LOAD FLAGS(7); /* Read, Write, Execute (RWX) for simplicity */
}

STACK_SIZE = 0x4000;   
NUM_CORES = 4;         

SECTIONS
{
    . = ORIGIN(RAM); 
    _kernel_start_vma = .;

    /* All loaded sections go into one segment: kernel_load */
    .text : ALIGN(4K) /* VMA of output .text section starts at _kernel_start_vma */
    {
        _text_output_section_start_vma = .;
        KEEP(*(.vectors))    
        KEEP(*(.text.boot))  
        
        KEEP(*(.init))       /* If crtbeginS.o or compiler generates .init */
        *(.text .text.*)     
        
        . = ALIGN(8);
        *(.rodata .rodata.*) 
        
        . = ALIGN(8);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*crtbeginS.o(.init_array)) 
        KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) .init_array*))
        KEEP (*(EXCLUDE_FILE (*crtendS.o) .init_array)) 
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(8);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*crtbeginS.o(.fini_array))
        KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) .fini_array*))
        KEEP (*(EXCLUDE_FILE (*crtendS.o) .fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);
        KEEP(*(.fini))
        
        KEEP(*(.eh_frame_hdr)) 
        KEEP(*(.eh_frame))     
        
        . = ALIGN(8);
        *(.plt) *(.plt.*) 
        
        . = ALIGN(8);
        *(.gnu.version) 
        *(.gnu.version_r)
        *(.rela.dyn)    
        *(.rela.plt)

        /* .data sections are now part of this single PHDR too */
        . = ALIGN(4K); /* Align VMA for where .data content will start */
        _data_section_start_vma_in_segment = .; 
        *(.data .data.*)    
        
        . = ALIGN(8);
        *(.got)       
        *(.got.plt)   
        
        . = ALIGN(8);
        *(.dynamic) 

        . = ALIGN(8); 
        _loaded_sections_end_vma = .; /* End of all file-backed content VMA */
    } > RAM :kernel_load /* All the above goes into the single kernel_load PHDR */
    /* LMA for the entire kernel_load PHDR starts at ORIGIN(RAM) */


    /* BSS section. VMA follows loaded sections. NOLOAD. */
    /* Still part of the kernel_load PHDR for memory allocation purposes by the loader. */
    /* Its VMA will be _loaded_sections_end_vma, aligned up. */
    . = _loaded_sections_end_vma; /* Ensure VMA for BSS starts after all loaded content */
    . = ALIGN(4K);                /* Align BSS start VMA */
    .bss (NOLOAD) : /* No AT() clause for NOLOAD */
    {
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(8);
        _bss_end = .;
    } > RAM :kernel_load /* BSS memory region is part of this segment for allocation purposes */
    _bss_size = _bss_end - _bss_start; 

    /* Stack section. VMA follows BSS. NOLOAD. */
    . = _bss_end; /* Start stack VMA after BSS */
    . = ALIGN(16); 
    .stacks (NOLOAD) : 
    {
        _stacks_start = .; 
        . = . + (STACK_SIZE * NUM_CORES);       
        _stacks_end = .;    
    } > RAM /* Not assigned to a PT_LOAD PHDR, just reserves RAM after loaded segments */
    
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
    }
}