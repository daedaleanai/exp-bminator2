# exp-bminator2

Experimental code for the 2nd generation (simplified) BMInator: an stm32l43x reads a BMI088 and a BME280 over SPI and writes the IMU and baro data out over USART.

The full interface description is documented in [the ICD](doc/ddln-bminator2-ICD.md)

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

TODO(lvd): Kaya version has PA3 USART2 RX debug serial console and PA11 bmi088 INT3 (EXTI11).


## DMA Mappings

| Device    | DMA | Ch  | Function          |
| --------- | --- | --- | ----------------- |
| SPI1 RX   | 1   | 2   | Read spi devices  |
| SPI1 TX   | 1   | 3   | Write spi devices |
| USART1 TX | 2   | 6   | Serial Output     |
| USART1 RX | 2   | 7   | Serial Input      |

## Connectors

Debug:  USART2TX, SWDIO, SWCLK, nRST, GND, 3V3
Host: USART1TX, USART1RX, PA15_TIMEPULSE, Power, GND
Heater:  Thermistor, GND, Heater10W+, Heater10W-


## Requirements

- [X] BMI088 Self test and configuration check.
- [X] BMI088 Sample Gyroscope at 2Khz, 250 deg/s full scale
- [X] BMI088 Sample Accelerometer at 1600Hz, 3G full scale
- [X] BMI088 Sample internal temperature at 1Hz
- [X] BME280 Sample environmental temperature, pressure, humidity at 1Hz
- [X] Watchdog monitors packets streaming regularily
- [X] Timepulse Sampled
- [ ] Heater power supply, control algorithm based on thermistor input and current sense.
- [ ] Heater state sampled at 8Hz
- [X] Samples reported over serial port cf ICD
- [X] Accept commands over serial port
- [X] Get/set BMI088, BME280
- [ ] Firmware upgradeable atomically (through write command)
- [ ] Send debug information in messages
- [ ] Box averaged gyroscope and accelerometer data output with temperature, ID and debug information on UART2
- [ ] Support multiplexing on UART2

implementation:
- [X] boot, clock, gpio, debug console for STM32L43x
- [X] SPI driver using DMA
- [X] USART driver using DMA
- [X] ADC driver

debug tools:
- [X] decoder
- [X] commander

calibration tool:
- [] data recorder with multiplexed UART support

TODO(lvd) 
- [ ] convert temperatures to correct units, check/fix stm temperature conversion
- [ ] make BMx registers all read only, make special registers to set accel/gyro sample rate and range
- [ ] test command flow more
- [X] time queue latencies
- [ ] implement stacktrace and assert for cortexm4

## Implementation

The code is based on a standalone bare metal environment with no dependencies on third party code. 
The basic design is that of interrupt handlers pushing data into queues which are handled by the main loop.

A guide to the source code is [here](doc/design.md).

## Debugging

the console can be displayed with 

    reset; (stty 115200 raw && cat) < /dev/cu.usbmodem1103

and looks like:

    SWREV:20629261
    CPUID:410fc241
    IDCODE:10016435.
    DEVID:20333937:48345016:003f0043
    RESET:04 PIN
    PPLSRC: MSI
    cal ts 1037 1378 vref 1661
    BMI088 Accel enabled.
    Starting gyro self test.
    Detected BMI088
    Starting accel self test.
    neutral xyz: 63 394 1278
    positive xyz: 11621 12865 13250
    negative xyz: -13285 -11659 -9329
    Gyro self test OK.
    BMI08x self test completed with 0 errors.
    T: 28202 26302 50
    P: 37156 -10744 3024 7987 39 -7 9900 -10230 4285
    H: 75 369 0 302 50 30
    395595 mainloop start

after which a periodic report looks like:

    uptime 21.396482  Vdd 3729 mV
    enqueued spiq:    77389 evq:      85 cmdq:  0 outq:    79036
    dropped  spiq:        0 evq:       0 cmdq:  0 outq:        0
    spi1   err tx:        0  rx:       0
    usart1 err tx:        0  rx:       0
    ------------- count - period - cumul - max
            IDLE  14986       66  960088   288
            WAIT
            MAIN   3688      271   39894  1100
          REPORT      1   999997    1088  1088
          ADCIRQ     32    31249      32     8
       USART1IRQ   5886      169    3798     4
     USART1TXDMA   3764      265    2235     4
     USART1RXDMA
         SHUTTER
         GYROIRQ   2038      490    6309     4
        ACCELIRQ   1644      608    5185     4
         8HZTICK      8   124999      10     2
        SPIRXDMA   3684      271    6701     3
        spiq_deq   3683      271  132143  1294
       spiq_xmit   3684      271  100780    51

    cmdbuf[18] 05 05 05 05 fc fc fc fc 00 00 00 10 23 00 01 00 e9 00
    CMDRX read 16 bytes at address 23000100

The period, cumulative and maximum times are in microseconds.
The cumulative time must be divided by the count to get the average time spent.
The spiq_xmit and spiq_deq timers measure the time spent between the enqueueing irq and the time the SPI transaction is done, resp dequeued in the main loop.
The IDLE and MAIN counters should add up to 1M microseconds per second, the REPORT until SPIRXDMA all run during the IDLE time, the WAIT runs during the MAIN timer.

The tools/ subdirectory contains two self contained Go programs, one that can read a serial stream
on stdin and prints a readable format on stdout, and one that can produce messages to set/get
registers.  

Sample usage:

    (stty 921600 raw && cat) < /dev/ttyXXX | go run tools/decode.go -m
    go run tools/encode.go -- register [value] > /dev/ttyXXX

output:

    4m20.000127625s
    8003    1 451.646390            ID0: [33958182 540227895]
    8004    1 451.771390            ID1: [1211387926 4128835]
    8020    1 451.396417           BARO: [24968 97725634]
    8021    1 451.396417          HUMID: [277121 0]
    8022    1 451.396392           TEMP: [63431 23500]
    8032 1625 452.116991       ACCEL_3G: [0.049 0.282 0.958]
    8039 2018 452.117417  GYRO_250DEG_S: [0.519 -4.227 -2.136]
    2023/12/09 14:14:29 Command response[32]: tag:0x17 status:0x00
    2023/12/09 14:14:29     data[0x00000010]: 00 0f 22 4a ff e2 00 34 fe 08 00 80 00 00 00 81



## Heater logic

Final tuning of the function will be done after tests, but the function to be implemented will have this format (Tth = temperature measured by the thermistor):

    For Tth ≤ X° C → P = 10 W (maximum power)
    For Tth > X° C → P(Tth) = -m*Tth + n (linear function where power decreases when Tth increases).

X = temperature to be determined iaw the requirement of avoiding icing conditions
m, n = parameters of the linear power function in relation to the measured temperature.
Ideally the function will reach P = 0 W when moisture is deemed to be no longer present based on the measured temperature (some margins are to be considered). 


## References

STM Reference documents:

- PM0214 Programming manual STM32 Cortex®-M4 MCUs and MPUs programming manual
- [RM0394](doc/rm0394-stm32l4xxxx.pdf) Reference manual STM32L41xxx/42xxx/43xxx/44xxx/45xxx/46xxx advanced Arm®-based 32-bit MCUs
- [DS11451](doc/stm32l432xx.pdf) STM32L432Kx Ultra-low-power Arm® Cortex®-M4 32-bit MCU+FPU, 100DMIPS, up to 256KB Flash, 64KB SRAM, USB FS, analog, audio
- [DS11453](doc/stm32l431xx.pdf) STM32L431xx Ultra-low-power Arm® Cortex®-M4 32-bit MCU+FPU, 100DMIPS, up to 256KB Flash, 64KB SRAM, analog, audio

Bosch reference documents:

- [BST-BMI088-DS001-13](doc/BMI088.pdf) BMI088 6-axis Motion Tracking for High-performance Applications
- [BST-BME280-DS001-18](doc/BME280.pdf) BME280 Combined humidity and pressure sensor
