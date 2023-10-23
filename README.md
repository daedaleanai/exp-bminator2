# exp-bminator2

Experimental code for the 2nd generation (simplified) BMInator: an stm32l43x reads a BMI088 and a BME280 over SPI and writes the IMU and baro data out over USART.

The full interface description is documented in [the ICD](doc/ddln-bminator2-ICD.md)

TODO(lvd) heater servo control logic

## STM32L431kx UFQFPN32 Pin Assignments:

| Pin  | Function   | DIR | Electrical           | Connected to                              |
| ---- | ---------- | --- | -------------------- | ----------------------------------------- |
| PA0  | CK_IN      | in  |                      | (reserved for 8Mhz external clock source) |
| PA1  | EXTINT A1  | in  | PullUp               | bmi088 INT1 (Accel) open drain/active low |
| PA2  | USART2 TX  | out | AF_PP 50MHz          | debug serial/ boot0 loader console RX     |
| PA3  | EXTINT A3  | in  | PullUp               | bmi088 INT3 (Gyro) open drain/active low  |
| PA4  | ADC1 IN9   | in  | Analog Input         | Heater Thermistor input                   |
| PA5  | GPIO PA5   | out | OUT_PP 2MHz          | Heater 24V  Enable                        |
| PA6  | ADC1 IN11  | in  | Analog Input         | Heater current sense                      |
| PA7  |            |     |                      |                                           |
| PA8  |            |     |                      |                                           |
| PA9  | USART1 TX  | out | AF_PP 50MHz          | host serial input                         |
| PA10 | USART1 RX  | in  | PullUp (5V tolerant) | host serial output                        |
| PA11 |            |     |                      | (reserved for CAN RX)                     |
| PA12 |            |     |                      | (reserved for CAN TX)                     |
| PA13 | JTMS/SWDIO |     |                      | debug SWD connector                       |
| PA14 | JTCK/SWCLK |     |                      | debug SWD connector                       |
| PA15 | TIM2 CH1   | in  | Input (5V tolerant)  | host reference time pulse                 |
| PB0  | GPIO B0    | out | OUT_PP 2MHz          | bmi088 CSB2 (Gyro)  active low            |
| PB1  | GPIO B1    | out | OUT_PP 2MHz          | bmi088 CSB1 (Accel) active low            |
| PB2  |            |     |                      | (not exposed on 32-pin package)           |
| PB3  | SPI1 SCK   | out | AF_PP 50MHz          | SPI sck                                   |
| PB4  | SPI1 MISO  | in  | PullUp               | SPI miso                                  |
| PB5  | SPI1 MOSI  | out | AF_PP 50MHz          | SPI mosi                                  |
| PB6  | GPIO B6    | out | OUT_PP 2MHz          | bme280 CSB  (Humid) active low            |
| PB7  |            |     |                      |                                           |

If desired, PA0/CK_IN can be used as an external clock source 4..48MHz, see
RMA0394 Section 6.1.1 p.182 'External Clock (HSE Bypass)'

The SPI1 bus connects SCK, MISO, MOSI to the BMI088 and BME280. They each require their own
separate CSB (active low).

## DMA Mappings

| Device    | DMA | Ch  | Function          |
| --------- | --- | --- | ----------------- |
| USART1 TX | 2   | 6   | Serial Output     |
| USART1 RX | 2   | 7   | Serial Input      |
| SPI1 RX   | 1   | 2   | Read spi devices  |
| SPI1 TX   | 1   | 3   | Write spi devices |

## Connectors
Debug:  USART2TX, SWDIO, SWCLK, nRST, GND, 3V3
Host: USART1TX, USART1RX, PA15_TIMEPULSE, (HSE Clock source), Power, GND
Heater:  Thermistor, GND, Heater10W+, Heater10W-


## Requirements

- [X] BMI088 Self test and configuration check.
- [X] BMI088 Sample Gyroscope at 2Khz, 250 deg/s full scale
- [X] BMI088 Sample Accelerometer at 1600Hz, 3G full scale
- [] BMI088 Sample internal temperature at 1Hz
- [X] BME280 Sample environmental temperature, pressure, humidity at 1Hz
- [] Watchdog monitors gyro, accel and env t/p/h are being sampled
- [] Timepulse Sampled
- [] Heater power supply, control algorithm based on thermistor input and current sense.
- [] Heater state sampled at 1Hz
- [] Samples reported over serial port cf ICD
- [] Accept commands over serial port
- [] Get/set BMI088, BME280
- [] Get/set eeprom

implementation:
- [X] boot, clock, gpio, debug console for STM32L43x
- [X] SPI driver using DMA
- [X] USART driver using DMA
- [] ADC driver

debug tools:
- [] decoder
- [] commander


## Debugging

the tools/ subdirectory contains two self contained Go programs, one that can read a serial stream
on stdin and prints a readable format on stdout, and one that can produce messages to set/get
registers.  

Sample usage:
    (stty 921600 raw && cat) < /dev/ttyXXX | go run tool/decode.go  
    go run tools/encode.go -- register [value] > /dev/ttyXXX


## References

STM Reference documents:

- PM0214 Programming manual STM32 Cortex®-M4 MCUs and MPUs programming manual
- RM0394 Reference manual STM32L41xxx/42xxx/43xxx/44xxx/45xxx/46xxx advanced Arm®-based 32-bit MCUs
- DS11451 STM32L432KB STM32L432KC Ultra-low-power Arm® Cortex®-M4 32-bit MCU+FPU, 100DMIPS, up to 256KB Flash, 64KB SRAM, USB FS, analog, audio
- DS11453 STM32L431xx Ultra-low-power Arm® Cortex®-M4 32-bit MCU+FPU, 100DMIPS, up to 256KB Flash, 64KB SRAM, analog, audio

Bosch reference documents:

- BST-BMI088-DS001-13 BMI088 6-axis Motion Tracking for High-performance Applications
- BST-BME280-DS001-18 BME280 Combined humidity and pressure sensor
