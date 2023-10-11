# exp-bminator2

Experimental code for the 2nd generation (simplified) BMInator: an stm32l43x reads a BMI088 and a BME280 over SPI and writes the IMU and baro data out over USART.

The full interface description is documented in [the ICD](doc/ddln-bminator2-ICD.md)

TODO(lvd) heater servo control logic

## STM32L431kx UFQFPN32 Pin Assignments:

| Pin  | Function   | DIR | Electrical           | Connected to                                 |
| ---- | ---------- | --- | -------------------- | -------------------------------------------- |
| PA0  | CK_IN      | in  |                      | (reserved for external clock source)         |
| PA1  | EXTINT A1  | in  | PullUp               | bmi088 INT1 (Accel) open drain/active low    |
| PA2  | USART2 TX  | out | AF_PP 50MHz          | debug serial/ boot0 loader console RX        |
| PA3  | EXTINT A3  | in  | PullUp               | bmi088 INT3 (Gyro) open drain/active low     |
| PA4  | ADC1 IN9   | in  | Analog Input         | Heater Thermistor input                      |
| PA5  | GPIO PA5   | out | OUT_PP 2MHz          | Heater 24V  Enable                           |
| PA6  |            |     |                      |                                              |
| PA7  | EXTINT A7  | in  | PullUp               | ina299 Alert open drain/active low           |
| PA8  |            |     |                      |                                              |
| PA9  | USART1 TX  | out | AF_PP 50MHz          | host serial input                            |
| PA10 | USART1 RX  | in  | PullUp (5V tolerant) | host serial output                           |
| PA11 |            |     |                      | (reserved for CAN RX)                        |
| PA12 |            |     |                      | (reserved for CAN TX)                        |
| PA13 | JTMS/SWDIO |     |                      | debug SWD connector                          |
| PA14 | JTCK/SWCLK |     |                      | debug SWD connector                          |
| PA15 | TIM2 CH1   | in  | Input (5V tolerant)  | host reference time pulse                    |
| PB0  | GPIO B0    | out | OUT_PP 2MHz          | bmi088 CSB2 (Gyro)  active low               |
| PB1  | GPIO B1    | out | OUT_PP 2MHz          | bmi088 CSB1 (Accel) active low               |
| PB2  |            |     |                      | (not exposed on 32-pin package)              |
| PB3  | SPI1 SCK   | out | AF_PP 50MHz          | SPI sck                                      |
| PB4  | SPI1 MISO  | in  | PullUp               | SPI miso                                     |
| PB5  | SPI1 MOSI  | out | AF_PP 50MHz          | SPI mosi                                     |
| PB6  | GPIO B6    | out | OUT_PP 2MHz          | bme280 CSB  (Humid) active low               |
| PB7  | GPIO B7    | out | OUT_PP 2MHz          | ina229 CSB (heater current sense) active low |

If desired, PA0/CK_IN can be used as an external clock source 4..48MHz, see
RMA0394 Section 6.1.1 p.182 'External Clock (HSE Bypass)'

The SPI1 bus connects SCK, MISO, MOSI to the BMI088, BME280 and the INA229.

## DMA Mappings

| Device    | DMA | Ch  | Function          |
| --------- | --- | --- | ----------------- |
| USART1 TX | 2   | 6   | Serial Output     |
| SPI1 RX   | 1   | 2   | Read spi devices  |
| SPI1 TX   | 1   | 3   | Write spi devices |

## Connectors
Debug:  USART2TX, SWDIO, SWCLK
Host: USART1TX, USART1RX, PA15_TIMEPULSE, (HSE Clock source)
Heater:  Thermistor, Heater24V

