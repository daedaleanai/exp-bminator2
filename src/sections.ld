/*
  This file assumes FLASHROM, CODE, SRAM2 have been defined before inclusion.
*/ 

_estack = ORIGIN(SRAM2) + LENGTH(SRAM2); /* top of the stack */

ENTRY(Reset_Handler)
EXTERN(vector_table)

SECTIONS
{

  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* vector_table */

    . = ALIGN(4);
  } >FLASHROM

  .text :
  {
    . = ALIGN(4);

    *(.text)
    *(.text.*)
    *(.rodata)       
    *(.rodata.*)

    . = ALIGN(4);
    _etext = .;

  } >FLASHROM

  _sidata = .;

  .data :
  {
    . = ALIGN(4);
    _sdata = .;

    *(.data)
    *(SORT_BY_ALIGNMENT(.data.*))

    . = ALIGN(4);
    _edata = .;
  
  } >SRAM1 AT> FLASHROM

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;

    *(.bss)
    *(SORT_BY_ALIGNMENT(.bss.*))
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;

  } >SRAM1

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
    *(.ARM.exidx)   /* not removing this messes with the data segment initialisation */
  }

}
