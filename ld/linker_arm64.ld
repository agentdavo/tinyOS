/* SPDX-License-Identifier: MIT OR Apache-2.0 */
/* Linker Script for miniOS ARM64 (Freestanding) */

ENTRY(_start) 

MEMORY
{
    RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 128M 
}

PHDRS
{
    kernel_rx PT_LOAD FLAGS(5); /* Read, Execute */
    kernel_rw PT_LOAD FLAGS(6); /* Read, Write */
}

SECTIONS
{
    . = ORIGIN(RAM); 
    _kernel_start_vma = .;

    /* ========================= Loadable TEXT Segment  =========================*/
    .text : ALIGN(4K) 
    {
        _text_section_start_vma = .;
        KEEP(*(.vectors))       /* First */
        KEEP(*(.text.boot))     /* _start is here */
        *(.text .text.*)        /* All other code */
        . = ALIGN(8);
        *(.rodata .rodata.*)    /* Read-only data */
        
        /* C++ Global Constructors - Init Array */
        . = ALIGN(8);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) .init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);

        /* C++ Global Destructors */
        . = ALIGN(8);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) .fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
        
        KEEP(*(.eh_frame_hdr)) /* Exception Handling - Unwind Info */
        KEEP(*(.eh_frame))     /* Needed for exception handling even if -fno-exceptions is used. *Might* pull in libgcc if kept. */

        /* Place GOT here if we are to force position independent code which is currently *NOT* happening */
        . = ALIGN(8);
        *(.plt) *(.plt.*) /* .plt entries */
        *(.rela.plt)       /* Relocations for PLT (if used) */
       
        . = ALIGN(8);
        _text_end = .; /* Symbol for end of text segment */
    } > RAM :kernel_rx_load

    _text_lma_end = LOADADDR(.text) + SIZEOF(.text); //End LMA of Text/ROData region (file)

    /* === Segment 2: Read-Write Data (in RAM) === */
	/* Start a data segment here. The loading memory address starts after _text_end. */

    . = _text_output_section_end_vma; /* Continue VMA from where the text segment ended in memory*/
    . = ALIGN(4K); /* Align VMA for beginning of the data segment */
    _data_segment_start_vma = .;

    .data : AT(_text_segment_lma_end) /* LMA follows directly after .text segment */
    {
        _data_section_start_vma = .;
        *(.data .data.*)    /* Initialized Data */
        . = ALIGN(8);
        *(.got)              /* Ensure GOT is present */
        *(.got.plt) /* Ensure PLT GOT if used for relocation is also present */

        . = ALIGN(8);
        *(.dynamic)          /* Dynamic linking data, if it exists */
        . = ALIGN(8);
        _data_section_end_vma = .; /* Symbol for end of the data section, aligned */
    } > RAM :kernel_rw_load
    _data_segment_size_in_file = SIZEOF(.data);


    /* === Segment 3: Stack Section (Allocated but not loaded). === */
	/* Starts after .data; not part of program header as it’s just a reservation in memory. */
    . = _data_section_end_vma; /* VMA Continue after .data  and all RW sections */
    . = ALIGN(4K);       /* Aligh current offset */

    .bss (NOLOAD) : {   /* BSS memory region for uninitialized data */
        _bss_start = .;   /* Label BSS start - ASM file uses this */
        *(.bss .bss.*)    /* All .bss */
        *(COMMON)          /* All COMMON */
        . = ALIGN(8);
        _bss_end = .; /* BSS end - needed for init by ASM*/
    } > RAM :kernel_rw_load /* BSS is part of the RW section - zero init */

    _bss_size = _bss_end - _bss_start;     /* calculate size */

	/* Stack memory region setup - No load, No PHDR, so no need to touch file offset, we leave it zero */
    . = ALIGN(16);          // Now start the stack, and ensure 16 byte alignment for security
    _stacks (NOLOAD) :   /* Each core is given its own stack, just stack area (64 KiB)*/
    {
		_stacks_start = .;
        . += (NUM_CORES * STACK_SIZE);
		_stacks_end = .;  /* each core stack is STACK_SIZE big, stack section is that many long */
    } > RAM

    _kernel_end = .;

	/* Other sections - discard, keep, just need to exist or get thrown to oblivion */
    /DISCARD/ : {
        *(.comment)        /* discard .comment section */
        *(.note*)          /* discard GNU notes */
        *(.ARM.attributes)  /* discard ARM arch. attributes */
        *(.interp)         /* discard dynamic linker section */
        *(.dynsym)         /* Should be no dynamic symbols present */
        *(.dynstr)         /* Should be no dynamic strings here */
        *(.gnu.hash)       /* Should be no GNU hash section */
        /* The relocation section will be discarded as well */
        *(.rela.dyn)  	  /* Section for runtime relocations */
    }
}