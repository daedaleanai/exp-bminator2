MEMORY
{
  FLASHROM (rx)  : ORIGIN = 0x08000000, LENGTH = 128K   /* stm32l4xxBxx . the stm32l4xxCxx have 256K  */
  SRAM1    (rw)  : ORIGIN = 0x20000000, LENGTH = 48K
  SRAM2    (rw)  : ORIGIN = 0x10000000, LENGTH = 16K
}

INCLUDE devs.ld

GPIO_ALL = GPIOA;               /* require for gpio2.[ch] */
UNIQUE_DEVICE_ID = 0x1FFF7590;  /* ref manual 47.1 */

_estack = ORIGIN(SRAM2) + LENGTH(SRAM2); /* top of the stack */

ENTRY(Reset_Handler)

SECTIONS
{

  .isr_vector :
  {

    . = ALIGN(4);
    PROVIDE (isr_vector_table_base = .);
    KEEP(*(.isr_vector)) /* Startup code */

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
    *(.data.*)

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
  }

}
