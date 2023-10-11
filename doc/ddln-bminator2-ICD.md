# DDLN BMInator 2 - Interface Control Document

DDLN-Doc: [Txxxx](https://p.daedalean.ai/Txx) 

**SIGNATURES**

| Role     | Name                    | Signature | Date |
| :------- | :---------------------- | :-------- | :--- |
| Author   | L. van Dijk - Daedalean |           |      |
| Reviewer |                         |           |      |
| Reviewer |                         |           |      |


**REVISION HISTORY**

| Date       | Revision | Change                                              | Author |
| :--------- | :------- | :-------------------------------------------------- | :----- |
| 2023-01-05 | 0.1      | Initial draft                                       | lvd    |
| 2023-01-16 | 0.2      | removed SoM and BRK, added multi-message framing    | lvd    |
| 2023-10-16 | 2.0      | Derived from DDLN-BMINATOR-KAYA-IRON253-ICD-v0.1    | lvd    |
|            |          | - removed references to Camera and CoaxPress        |        |
|            |          | - replaced these with 'Host Unit' where appropriate |        |
|            |          | - changed BMP288 to BME280                          |        |
|            |          | - Baro message frequency 12.5 to 1Hz                |        |
|            |          | - added HUMID message type                          |        |
|            |          | -  change pinout and PCB descriprions               |        |
|            |          | -  facility to read/write BMI and BME registers     |        |
|            |          | and a flash page mapped to genicam register         |        |
|            |          | read/write messages.                                |        |

## Introduction

### Purpose

The purpose of this document is to define the interface between the Daedalean BMInator v2 board and a host unit.

### Scope

#### Document Identification

This document is referenced as DDLN-BMINATOR-ICD-v2.0.

#### Document Scope

This document contains all information required to interface to the Daedalean BMInator v2 reference implementation.

### Applicable Documents

#### External Documents

**BMI088 Datasheet** [BST-BMI-088-DS001](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
**BME280 Datasheet** [BST-BME-280-DS002](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
**STM32L4xx Reference Manual** [RM0394](https://www.st.com/resource/en/reference_manual/rm0394-stm32l41xxx42xxx43xxx44xxx45xxx46xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
**STM32L431cb Datasheet** [STM32L431](https://www.st.com/resource/en/datasheet/stm32l431cb.pdf)
**STM32L432kc Datasheet**  [STM32L432](https://www.st.com/resource/en/datasheet/stm32l432kc.pdf)


#### Internal Documents

n/a

### Nomenclature and Description of Terms

#### Description of Terms

**BMInator v2** 
A small PCB with an STM32L43x microcontroller, a Bosch BMI088 and BME280 sensor and firmware to read out the sensor and output the data over a serial line.

**Host Unit** 
A system interfacing to the BMInator v2, (e.g. a Host Unit).

**MUST** A requirement for the the system.

## System Overview

The Daedalean BMInator v2 contains a Bosch BMI088 MEMS IMU to measure 6-dof motion at up to 1600 Hz(accelerometer) and 2000 Hz(gyroscope) and a Bosch BME280 which senses environmental temperature, pressure and humidity.

A Host Unit must be able to obtain the IMU and environmental data streams from the BMInator and be able to correlate them in time to it's own time reference.  The Host Unit must also be able to read and control settings in the sensors, and store and retrieve up to a kilobyte of data in it's non volatile memory.  

To this end the BMInator is physically, electically and logically connected to the Host Unit, which can send a time pulse and serial commands, and receives a timestamped datastream with the measurements, and responses to the commands.


### Functional overview

This document defines the physical, electrical and data interfaces carried over the connections from BMInator to Host Unit to achieve the following functionality:

1. The BMInator is powered from the Host Unit,
2. The BMInator receives a reference time pulse from the Host Unit in order to correlate these events to its internal timer,
3. The BMInator can receive commands over a serial port from the Host Unit,
4. The BMInator transmits a stream of messages with IMU, health and timing information, and the responses to the commands over a serial port to the Host Unit.

## Physical interface

The BMInator is a PCB of XXxXXmm, with YY Mx screw holes and 2 connectors according to the following image:

![bminator pcb front copper image](layout.png){ height=10cm }

BMInator Connector Jx(DEBUG) is not used to interface to the Host Unit, but should be kept accessible to enable BMInator firmware upgrades.  

BMInator Connector Jy(IO) is a XXXXX 1xN pin connector pitch 0.024" (0.60mm), with pin 1 outermost on the board. 

The BMI088 z-axis is orthogonal to the PCB plane.  The PCB has a marking indicating the XY-axes of the BMI088.

## Electrical interface


The BMInator Connector Jx (IO) has the following pinout

| Pin | Name       | Function       | Direction | Electrical        | Connected to Host Unit signal |
| --: | :--------- | :------------- | :-------- | :---------------- | :---------------------------- |
|   1 | GND        |                |           |                   | ground                        |
|   4 | +3.3V      |                | B <- H    | power supply 3.3V | +3.3V supply                  |
|   5 | Time Pulse | PA15 TIM1 CH2  | B <- H    | input 5V tolerant | Reference Time Pulse          |
|   6 | Serial TX  | PA9 USART1 TX  | B -> H    | AF_PP 10MHz 3.3V  | serial RX                     |
|   6 | Serial RX  | PA10 USART1 RX | B <- H    | Input 5V tolerant | serial TX 5V                  |
BMInator Connector Jy(IO) pinout.

Direction B->H means the signal or power flows from BMInator to Host Unit, B <- H means from Host Unit to BMInator.


The BMInator MUST be supplied by the Host Unit with (TODO power).

The BMInator MUST be supplied by the Host Unit with a digital reference time pulse not exceeding 5V.

The BMInator MUST supply the Host Unit with a serial port digital signal of 3.3V, with a baudrate of 921600 Baud. 

The electrical characteristics of PA9, PA10 and PA15 are described in the STM32L43x Datasheet section 5.

## Logical Interface

### Host Unit Time Pulse Reference Signal

The time pulse reference is a digital signal provided by the Host Unit to the BMInator, which times the level changes.
The BMInator will emit messages correlating the raise and fall events to its internal clock with a precision better than 1 microsecond.

### Serial Signal

The BMInator MUST output and input a serial stream of 921600 Baud, with 8 data bits, No parity and 1 Stop bit over J2(IO) pins X/Y (uC function PA9/PA10) for an effective Byte rate of 92160 Bytes per second.  In accordance with standard serial bit stream protocols, the LSB is transmitted first on the wire.


### Packets and Messages

The BMInator send messages over the serial port framed into packets of up to 1024 bytes and up to 50 messages.

The BMInator receives messages over the serial port, framed one message at at time in a different format than the output.

#### Output Packet Format

| Content             | Description                                                                     |
| :------------------ | :------------------------------------------------------------------------------ |
| 0x49 0x52 0x4F 0x4E | a magic header consisting of the ASCII characters 'I','R','O','N' in that order |
| uint16              | a big endian 16 bit byte count of the following messages                        |
| messages            | a sequence of messages as defined below                                         |
| uint16              | the sum over all message bytes as a big-endian 16 bit number                    |

(The checksum is a simple sum, which is not very resilient against common types of serial errors. consider replacing with a CRC16 in the future)

#### Output Message Format

The BMInator produces messages in the format of a CoaXpress Event Message, for ease of forwarding encapsulated in a CoaXPress Event Packet in certain applications.
All relevant aspects are reproduced here, so the CoaXpress standard need not be consulted.

Transmission is organized in groups of 4 characters called a word, labelled P0, P1, P2, P3, transmitted in that order on the wire.
When 16, 24 or 32 bit quantities are encoded in a word, the order of the bytes within the word is big-endian.  
A 64 bit quantity is transmitted as 2 words, in big-endian order

| Value    | P0       | P1       | P2       | P3       |
| :------- | :------- | :------- | :------- | :------- |
| 4x8 bit  | b0[0:7]  | b1[0:7]  | b2[0:7]  | b3[0:7]  |
| 2x16 bit | h0[8:15] | h0[0:7]  | h1[8:15] | h1[0:7]  |
| 1x32 bit | w[24:31] | w[16:23] | w[8:15]  | w[0:7]   |
| 1x64 bit | v[56:63] | v[48:55] | v[40:47] | v[32:39] |
|          | v[24:31] | v[16:23] | v[8:15]  | v[0:7]   |
Example big-endian encoding of 8, 16 and 32 bit values in a Word.

With these definitions, the BMInator formats the output over the serial port as follows:

| Word     | Content          | Description                                                                                      |
| :------- | :--------------- | :----------------------------------------------------------------------------------------------- |
| 0        | EventSize        | 16 bit value in bits 31:16 (characters P0, P1) representing the                                  |
|          |                  | number of data bytes B for this event message, i.e. over words 0 to M+2.                         |
|          | Namespace        | 2 bit field in bits 15:14 0b10 specifying event namespace, set to 2 = Device Specific Event ID.  |
|          | Reserved         | 2 bit field in bits 13:12 set to 0.                                                              |
|          | EventID          | 12 bit field in bits 11:0 specifying the event source (set according to message type, see below. |
| 1        | Timestamp(63:32) | 64 bit value representing the value of the Device time when the Device detected the situation    |
| 2        | Timestamp(31:0)  | resulting in this event. (see note)                                                              |
| 3 to M+2 | Data             | zero padded to word size.                                                                        |
Event message format -- cf CoaXPress Standard Version 2.1 p.68 Table 29.

Note: The Timestamp field is filled in by the BMInator microcontroller referenced to its internal clock, with no relation to the Host Unit's internal clock. Strictly speaking this is not conform the CoaXPress standard.

#### Output Message payloads


The BMInator generates messages with the following 12-bit EventID and contents

|    ID |   Size | Word 0 (hex) | Freq. (Hz) | Name                       | Type        | Content                                                        |
| ----: | -----: | -----------: | ---------: | :------------------------- | :---------- | :------------------------------------------------------------- |
| 0x003 |     20 |  00 14 80 03 |          1 | ID0                        | [2]uint32   | software revision:UUID[2]                                      |
| 0x004 |     20 |  00 14 80 04 |          1 | ID1                        | [2]uint32   | microcontroller UUID[1]:UUID[0]                                |
| 0x020 |     20 |  00 14 80 20 |          1 | BARO                       | [2]uint32   | BME280 temperature (milli Kelvin) and pressure (milli Pascal)  |
| 0x020 |     20 |  00 14 80 21 |          1 | HUMID                      | [2]uint32   | BME280 humidity measurement                                    |
| 0x022 |     20 |  00 14 80 22 |          1 | TEMP                       | [2]uint32   | BMI088 temperature, STM32L43x temperature (milli Kelvin)       |
| 0x023 |     20 |  00 14 80 23 |        FPS | time pulse reference_OPEN  | uint64      | Host Unit time pulse reference raise event, payload is counter |
| 0x025 |     20 |  00 14 80 25 |        FPS | time pulse reference_CLOSE | uint64      | Host Unit time pulse reference fall event, payload is counter  |
| 0x032 |     20 |  00 14 80 32 |       1600 | ACCEL_3G                   | 4x int16    | BMI088 raw accelerometer reading x,y,z + 0x0000 padding        |
| 0x033 |        |  00 14 80 33 |            | ACCEL_6G                   |             | see note                                                       |
| 0x034 |        |  00 14 80 34 |            | ACCEL_12G                  |             | see note                                                       |
| 0x035 |        |  00 14 80 35 |            | ACCEL_24G                  |             | see note                                                       |
| 0x038 |     20 |  00 14 80 38 |       2000 | GYRO_125DEG_S              | 4x int16    | BMI088 raw gyroscope reading x,y,z + 0x0000 padding            |
| 0x039 |        |  00 14 80 39 |            | GYRO_250DEG_S              |             | see note                                                       |
| 0x03a |        |  00 14 80 3a |            | GYRO_500DEG_S              |             | see note                                                       |
| 0x03b |        |  00 14 80 3b |            | GYRO_1000DEG_S             |             | see note                                                       |
| 0x03c |        |  00 14 80 3c |            | GYRO_2000DEG_S             |             | see note                                                       |
| 0x040 |     20 |  00 14 80 40 |            | RWREGS_RESP_OK             | [2]uint32   | address, value  for succesful read/write register command      |
| 0x048 |        |  00 14 80 48 |            | RWREGS_RESP_FAIL           | [2]uint32   | address, reason for failed r/w-register or readmem command     |

Event Message types. 

Note: the BMInator will send only one of the message types 0x032..0x35 and of 0x038..0x3c, depending on the current sensitiviy setting of its Gyroscope and Accelerometer components.  
The eventid encodes the full scale value of the int16 numbers as per the BMI085 datasheet.

Event types 0x23 and 0x25 have a timestamp recorded in the Event Message header as described above. They are sent at the Host Unit frame rate. Their payload is a continuously incremented counter.

The RWREGS_RESP_ messages are responses to the READ/WRITE Register/Memory commands received on the serial input (see below).  They will be interleaved with the datastream. (TODO: maybe they should go into a separate packet so that the host can more easily separate them?)

The bandwidth will be dominated by the 3600Hz messages of 20 bytes each, or 72 kB/s, taking up 720 kBaud on the serial line, about or 80%. 

The two padding bytes in the ACCEL and GYRO messages may be set to zero or may be used for an error correction code on th payload, TBD(lvd).

The streaming messages are all exactly 20 bytes in length to ease the framing and de-framing as described above.  

#### Input Message Format

proposal:
0x7F  sync character, allows auto baud detection in uC
len   1 byte lenght of message, up to 16 bytes
4 bytes address, top bit is 1 for writes, 0 for reads
4 bytes value (only present for writes)
checksum?

address space for read writes: genicam specifies read/write addresses in units of bytes but 32-bit aligned so last two addres bits always zero.
the SPI devices have an 8 bit address space of bytes.

prefix: 16 bits to choose so as to not conflict with existing GENICAM addresses, 4 bits to select SPI device (1: BMI088, 2:BME280, ...), 8 bits to select byte register within SPI device, two trailing zero bits to make genicam compatible

for eeprom: unique prefix + 10 bits, last two zero, to map to up to 1kb of uC flash rom.


