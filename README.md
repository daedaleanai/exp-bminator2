# exp-bminator2

Experimental code for the 2nd generation simplified BMInator: an stm32l431 reads a BMI088 and a BME280 over SPI and writes the IMU and baro data out over USART2.

The full interface description is documented in [the ICD](doc/doc/ddln-bminator2-IDC.md)


## STM32L431cb LQFP48 and STM32L432kc UFQFPN32 Pin Assignments:

| V   | Pin  | Function   | DIR | Electrical  | Connected to                                     |
| --- | ---- | ---------- | --- | ----------- | ------------------------------------------------ |
|     | PA0  | EXTINT A0  | in  | PullUp      | bmi088 INT1 (Accel) open drain/active low        |
|     | PA1  | EXTINT A1  | in  | PullUp      | bmi088 INT3 (Gyro)  open drain/active low        |
|     | PA2  | USART2 TX  | out | AF_PP 50MHz | camera fpga serial input                         |
|     | PA3  | USART2 RX  | in  | PullUp      | camera fpga serial output                        |
| P   | PA4  | SPI3 NSS   | out | AF_PP 50MHz | bme280 CSB  (Baro)  active low                   |
|     | PA5  | SPI1 SCK   | out | AF_PP 50MHz | bmi088 SPI sck                                   |
|     | PA6  | SPI1 MISO  | in  | PullUp      | bmi088 SPI miso (sdo2/sdo1)                      |
|     | PA7  | SPI1 MOSI  | out | AF_PP 50MHz | bmi088 SPI mosi (sdi)                            |
|     | PA8  |            |     |             |                                                  |
|     | PA9  | USART1 TX  | out | AF_PP 10MHz | serial TX / boot0 loader console RX              |
|     | PA10 |            |     |             |                                                  |
|     | PA11 |            |     |             |                                                  |
|     | PA12 |            |     |             |                                                  |
|     | PA13 | JTMS/SWDIO |     |             | SWD connector                                    |
|     | PA14 | JTCK/SWCLK |     |             | SWD connector                                    |
| P   | PA15 | TIM2 CH1   | in  | Float       | camera shutter output (5V tolerant)  (note: CH1 not CH3!) |
|     | PB0  | GPIO B0    | out | OUT_PP 2MHz | bmi088 CSB2 (Gyro)  active low                   |
|     | PB1  | GPIO B1    | out | OUT_PP 2MHz | bmi088 CSB1 (Accel) active low                   |
|     | PB2  |            |     |             |                                                  |
| P   | PB3  | SPI3 SCK   | out | AF_PP 50MHz | bme280 SPI sck                                   |
| P   | PB4  | SPI3 MISO  | in  | PullUp      | bme280 SPI miso (sdo)                            |
| P   | PB5  | SPI3 MOSI  | out | AF_PP 50MHz | bme280 SPI mosi (sdi)                            |
|     | PB6  |            |     |             |                                                  |
|     | PB7  |            |     |             |                                                  |
|     | PB8  |            |     |             |                                                  |
|     | PB9  |            |     |             |                                                  |
| X   | PB10 | TIM2 CH3   | in  | Float       | camera shutter output (5V tolerant)                       |
| X   | PB12 | SPI2 NSS   | out | AF_PP 50MHz | bme280 CSB  (Baro)  active low                   |
| X   | PB13 | SPI2 SCK   | out | AF_PP 50MHz | bme280 SPI sck                                   |
| X   | PB14 | SPI2 MISO  | in  | PullUp      | bme280 SPI miso (sdo)                            |
| X   | PB15 | SPI2 MOSI  | out | AF_PP 50MHz | bme280 SPI mosi (sdi)                            |

P: STM32L432kc 32-pin Prototype version only
X: STM32L431cb 48-pin Production version only

Since the STM32L432kc UFQFPN32 package does not have pins beyond PB7,
the bme280 is connected to SPI3 on PA4/PB3,4,5 and the camera shutter
input is mapped to TIM2 CH1 on PA15.

## DMA Mappings

|     | Device    | DMA | Ch  | Function      |
| --- | --------- | --- | --- | ------------- |
|     | USART2 TX | 1   | 7   | Serial Output |
|     | SPI1 RX   | 2   | 3   | Read BMI088   |
|     | SPI1 TX   | 2   | 4   | Write BMI088  |
| X   | SPI2 RX   | 1   | 4   | Read BME280   |
| X   | SPI2 TX   | 1   | 5   | Write BME280  |
| P   | SPI3 RX   | 2   | 1   | Read BME280   |
| P   | SPI3 TX   | 2   | 2   | Write BME280  |
