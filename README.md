# exp-bminator2

Experimental code for the 2nd generation (simplified) BMInator: an stm32l43x reads a BMI088 and a BME280 over SPI and writes the IMU and baro data out over USART.

The full interface description is documented in [the ICD](doc/doc/ddln-bminator2-ICD.md)

TODO(lvd) heater servo control logic

## STM32L431kb and STM32L432kc UFQFPN32 Pin Assignments:

| Pin  | Function   | DIR | Electrical  | Connected to                              |
| ---- | ---------- | --- | ----------- | ----------------------------------------- |
| PA0  | EXTINT A0  | in  | PullUp      | bmi088 INT1 (Accel) open drain/active low |
| PA1  | EXTINT A1  | in  | PullUp      | bmi088 INT3 (Gyro)  open drain/active low |
| PA2  | USART2 TX  | out | AF_PP 50MHz | host serial input                         |
| PA3  | USART2 RX  | in  | PullUp      | host serial output                        |
| PA3  | ADC1 IN8   | in  | Analog      | Heater Thermistor input                   |
| PA4  | GPIO A4    | out | AF_PP 50MHz | bme280 CSB  (Humid) active low            |
| PA5  | SPI1 SCK   | out | AF_PP 50MHz | bmi088/bme280/... SPI sck                     |
| PA6  | SPI1 MISO  | in  | PullUp      | bmi088/bme280/... SPI miso (sdo2/sdo1)        |
| PA7  | SPI1 MOSI  | out | AF_PP 50MHz | bmi088/bme280/... SPI mosi (sdi)              |
| PA8  |            |     |             |                                           |
| PA9  | USART1 TX  | out | AF_PP 10MHz | debug serial/ boot0 loader console RX     |
| PA10 |            |     |             |                                           |
| PA11 |            |     |             |                                           |
| PA12 |            |     |             |                                           |
| PA13 | JTMS/SWDIO |     |             | debug SWD connector                       |
| PA14 | JTCK/SWCLK |     |             | debug SWD connector                       |
| PA15 | TIM2 CH1   | in  | Float       | reference time pulse (5V tolerant)        |
| PA15 | GPIO PA15  | out | OUT_PP 2MHz | Heater 24V  Enable                        |
| PB0  | GPIO B0    | out | OUT_PP 2MHz | bmi088 CSB2 (Gyro)  active low            |
| PB1  | GPIO B1    | out | OUT_PP 2MHz | bmi088 CSB1 (Accel) active low            |
| PB2  |            |     |             |                                           |
| PB3  | SPI3 SCK   | out | AF_PP 50MHz | bme280 SPI sck                            |
| PB4  | SPI3 MISO  | in  | PullUp      | bme280 SPI miso (sdo)                     |
| PB5  | SPI3 MOSI  | out | AF_PP 50MHz | bme280 SPI mosi (sdi)                     |
| PB5  |            |     |             | 24 V Supply ctl /int                      |
| PB6  |            |     |             |                                           |
| PB7  |            |     |             |                                           |



## DMA Mappings

| Device    | DMA | Ch  | Function            |
| --------- | --- | --- | ------------------- |
| USART2 TX | 1   | 7   | Serial Output       |
| SPI1 RX   | 2   | 3   | Read BMI088/BME280  |
| SPI1 TX   | 2   | 4   | Write BMI088/BME280 |
