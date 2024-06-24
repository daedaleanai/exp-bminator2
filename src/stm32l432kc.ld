/* stm32l43xxCxx have 256k flash and 48+16k ram. */
MEMORY
{
  FLASHROM (rx)  : ORIGIN = 0x08000000, LENGTH = 256K  
  SRAM1    (rw)  : ORIGIN = 0x20000000, LENGTH = 48K 
  SRAM2    (rw)  : ORIGIN = 0x10000000, LENGTH = 16K /* aliased at 0x2000c000 */
}

INCLUDE devs.ld

GPIO_ALL = GPIOA;               /* required for gpio2.[ch] */
UNIQUE_DEVICE_ID = 0x1FFF7590;  /* ref manual 47.1 */

TS_CAL1 = 0x1FFF75A8;  /* ref man 16.4.32, datasheet 3.15.1 */
TS_CAL2 = 0x1FFF75CA;  
VREFINT = 0x1FFF75AA; /* ref man 16.4.33, datasheet 3.15.2 */

INCLUDE sections.ld
