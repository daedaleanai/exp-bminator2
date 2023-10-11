# exp-bminator2

Experimental code for the 2nd generation (simplified) BMInator: an stm32l43x reads a BMI088 and a BME280 over SPI and writes the IMU and baro data out over USART.

The full interface description is documented in [the ICD](doc/ddln-bminator2-ICD.md)

TODO(lvd) heater servo control logic

## STM32L431kx UFQFPN32 Pin Assignments:

| Pin  | Function   | DIR | Electrical           | Connected to                              |
| ---- | ---------- | --- | -------------------- | ----------------------------------------- |
| PA0  | CK_IN      | in  |                      | (reserved for external clock source)      |
| PA1  | EXTINT A1  | in  | PullUp               | bmi088 INT1 (Accel) open drain/active low |
| PA2  | USART2 TX  | out | AF_PP 50MHz          | debug serial/ boot0 loader console RX     |
| PA3  | EXTINT A3  | in  | PullUp               | bmi088 INT3 (Gyro) open drain/active low  |
| PA4  | ADC1 IN9   | in  | Analog Input         | Heater Thermistor input                   |
| PA5  | SPI1 SCK   | out | AF_PP 50MHz          | bmi088/bme280/... SPI sck                 |
| PA6  | SPI1 MISO  | in  | PullUp               | bmi088/bme280/... SPI miso (sdo2/sdo1)    |
| PA7  | SPI1 MOSI  | out | AF_PP 50MHz          | bmi088/bme280/... SPI mosi (sdi)          |
| PA8  |            |     |                      |                                           |
| PA9  | USART1 TX  | out | AF_PP 50MHz          | host serial input                         |
| PA10 | USART1 RX  | in  | PullUp (5V tolerant) | host serial output                        |
| PA11 |            |     |                      | (reserved for CAN RX)                     |
| PA12 |            |     |                      | (reserved for CAN TX)                     |
| PA13 | JTMS/SWDIO |     |                      | debug SWD connector                       |
| PA14 | JTCK/SWCLK |     |                      | debug SWD connector                       |
| PA15 | TIM2 CH1   | in  | Input (5V tolerant)  | host reference time pulse                 |
| PA15 | GPIO PA15  | out | OUT_PP 2MHz          | Heater 24V  Enable                        |
| PB0  | GPIO B0    | out | OUT_PP 2MHz          | bmi088 CSB2 (Gyro)  active low            |
| PB1  | GPIO B1    | out | OUT_PP 2MHz          | bmi088 CSB1 (Accel) active low            |
| PB2  |            |     |                      | (not exposed on 32-pin package)           |
| PB3  | GPIO B3    | out | OUT_PP 2MHz          | bme280 CSB  (Humid) active low            |
| PB4  | GPIO B4    | out | OUT_PP 2MHz          | heater current sense CSB active low       |
| PB5  |            |     |                      |                                           |
| PB6  |            |     |                      |                                           |
| PB7  |            |     |                      |                                           |

If desired, PA0/CK_IN can be used as an external clock source 4..48MHz, see
RMA0394 Section 6.1.1 p.182 'External Clock (HSE Bypass)'

## DMA Mappings

| Device    | DMA | Ch  | Function            |
| --------- | --- | --- | ------------------- |
| USART1 TX | 2   | 6   | Serial Output       |
| SPI1 RX   | 1   | 2   | Read BMI088/BME280  |
| SPI1 TX   | 1   | 3   | Write BMI088/BME280 |

## Connectors
Debug:  USART2TX, SWDIO, SWCLK
Host: USART1TX, USART1RX, PA15_TIMEPULSE, (HSE Clock source)
Heater:  Thermistor, Heater24V

