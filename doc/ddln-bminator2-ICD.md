# DDLN BMInator 2 - Interface Control Document

DDLN-Doc: [Txxxx](https://p.daedalean.ai/Txx) 

**SIGNATURES**

| Role     | Name                    | Signature | Date |
| :------- | :---------------------- | :-------- | :--- |
| Author   | L. van Dijk - Daedalean |           |      |
| Reviewer |                         |           |      |
| Reviewer |                         |           |      |


**REVISION HISTORY**

| Date       | Revision | Change                                           | Author |
| :--------- | :------- | :----------------------------------------------- | :----- |
| 2023-01-05 | 0.1      | Initial draft                                    | lvd    |
| 2023-01-16 | 0.2      | removed SoM and BRK, added multi-message framing | lvd    |
| 2023-10-16 | 2.0      | Derived from DDLN-BMINATOR-KAYA-IRON253-ICD-v0.1 | lvd    |

## Introduction

### Purpose

The purpose of this document is to define the interface between the Daedalean BMInator v2 board and a hosting device, typically a camera.

### Scope

#### Document Identification

This document is referenced as DDLN-BMINATOR-ICD-v2.0.

#### Document Scope

This document contains all information required to interface to the Daedalean BMInator v2 reference implementation.

### Applicable Documents

#### External Documents

- **BMI088 Datasheet** [BST-BMI-088-DS001](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
- **BME280 Datasheet** [BST-BME-280-DS002](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- **STM32L4xx Reference Manual** [RM0394](https://www.st.com/resource/en/reference_manual/rm0394-stm32l41xxx42xxx43xxx44xxx45xxx46xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- **STM32L431cb Datasheet** [STM32L431](https://www.st.com/resource/en/datasheet/stm32l431cb.pdf)
- **STM32L432kc Datasheet**  [STM32L432](https://www.st.com/resource/en/datasheet/stm32l432kc.pdf)
- **CoaXPress Standard Version 2.1** [JIIA CXP-001-2021](https://TODO)

#### Internal Documents

n/a

### Nomenclature and Description of Terms

#### Description of Terms

**BMInator v2** 
A small PCB with an STM32L43x microcontroller, a Bosch BMI088 and BME280 sensor and firmware to read out the sensor and output the data over a serial line.  In addition the device contains a current source and analog inputs to control a heater element.

**Host Unit** 
A system interfacing to the BMInator v2, (e.g. a CoaXPress camera).

**MUST** A requirement for the the system.

## System Overview

The Daedalean BMInator v2 contains a Bosch BMI088 MEMS IMU to measure 6-dof motion at up to 1600 Hz(accelerometer) and 2000 Hz(gyroscope) and a Bosch BME280 which senses environmental temperature, pressure and humidity.

A Host Unit must be able to obtain the IMU and environmental data streams from the BMInator and be able to correlate them in time to it's own time reference.  The Host Unit must also be able to read and control settings in the sensors, and store and retrieve up to a kilobyte of data in it's non volatile memory.

In addition the device contains a current source to supply a heater element and an analog input to sense temperature using a thermistor.  The Host Unit can send commands to set the desired temperature.

To this end the BMInator is physically, electically and logically connected to the Host Unit, which can send a time pulse and serial commands, and receives a timestamped datastream with the measurements, and responses to the commands.


### Functional overview

This document defines the physical, electrical and data interfaces carried over the connections from BMInator to Host Unit to achieve the following functionality:

1. The BMInator is powered from the Host Unit,
2. The BMInator receives a reference time pulse from the Host Unit in order to correlate these events to its internal timer,
3. The BMInator can receive commands over a serial port from the Host Unit,
4. The BMInator transmits a stream of event messages with IMU, health and timing information, over a serial port to the Host Unit.
5. The BMInator transmits responses to commands over the same serial port. 
6. The BMInator supplies up to 10W of power to a heater element to maintain a preset temperature of an external heating element.

## Physical interface

A reference implementation of the BMInator is a PCB of XXxXXmm, with YY Mx screw holes and 3 connectors according to the following image:

![bminator pcb front copper image](layout.png){ height=10cm }

TODO(ph) update with image of the actual pcb

BMInator Connector Jx(DEBUG) is used to perform BMInator firmware upgrades and debug during firmware development.  It is typically not connected to the Host Unit.

BMInator Connector Jy(IO) is a XXXXX 1xN pin connector pitch 0.024" (0.60mm), with pin 1 outermost on the board. 

BMInator Connector Jz(Heater) a XXXXX 1xN pin connector pitch 0.024" (0.60mm), with pin 1 outermost on the board. 

The BMI088 z-axis is orthogonal to the PCB plane.  The PCB has a marking indicating the XY-axes of the BMI088.

## Electrical interface

TODO(ph) update with pinout of the actual implementation

| Pin | Name      | Function      | Direction | Electrical        | Connected to debugger |
| --: | :-------- | :------------ | :-------- | :---------------- | :-------------------- |
|   1 | Serial TX | PA2 USART2 TX | B -> D    | AF_PP 10MHz 3.3V  | serial RX             |
|   2 | nRST      | nRST          | B <- D    |                   | reset                 |
|   3 | SWDIO     | PA13 SWDIO    | B <-> D   |                   | debug data            |
|   4 | GND       |               |           |                   | ground                |
|   5 | SWCLK     | PA14 SWCLK    | B <- D    |                   | debug clock           |
|   6 | +3.3V     |               | B <- D    | power supply 3.3V | +3.3V supply          |

BMInator Connector Jx(DEBUG) pinout.

Direction B->D means the signal or power flows from BMInator to debugger, B <- D means v.v.
The electrical characteristics of PA2, PA13, PA14 and nRST are described in the STM32L43x Datasheet section 5.

The BMInator Connector Jy (IO) has the following pinout

| Pin | Name       | Function       | Direction | Electrical        | Connected to Host Unit signal |
| --: | :--------- | :------------- | :-------- | :---------------- | :---------------------------- |
|   1 | GND        |                |           |                   | ground                        |
|   2 | +3.3V      |                | B <- H    | power supply 3.3V | +3.3V supply                  |
|   3 | Time Pulse | PA15 TIM1 CH2  | B <- H    | input 5V tolerant | Reference Time Pulse          |
|   4 | Serial TX  | PA9 USART1 TX  | B -> H    | AF_PP 10MHz 3.3V  | serial RX                     |
|   5 | Serial RX  | PA10 USART1 RX | B <- H    | Input 5V tolerant | serial TX 5V                  |

BMInator Connector Jy(IO) pinout.

Direction B->H means the signal or power flows from BMInator to Host Unit, B <- H means from Host Unit to BMInator.

The BMInator MUST be supplied by the Host Unit with (TODO power).

The BMInator MUST be supplied by the Host Unit with a digital reference time pulse not exceeding 5V.

The BMInator MUST supply the Host Unit with a serial port digital signal of 3.3V, with a baudrate of 921600 Baud. 

The electrical characteristics of PA9, PA10 and PA15 are described in the STM32L43x Datasheet section 5.

## Logical Interface

### Host Unit Time Pulse Reference Signal

The time pulse reference is a digital signal provided by the Host Unit to the BMInator, which times the level changes.
The BMInator will emit messages correlating the raise and fall events to its internal clock with a resolution of 1 microsecond.

This function can be used to correlate the measurement data with a camera shutter opening and closing.

### Serial Signal

The BMInator MUST output and input a serial stream of 921600 Baud, with 8 data bits, No parity and 1 Stop bit over J2(IO) pins X/Y (uC function PA9/PA10) for an effective Byte rate of 92160 Bytes per second.  In accordance with standard serial bit stream protocols, the LSB is transmitted first on the wire.

### Packets and Messages

The BMInator sends and receives data grouped in packets of up to 1024 bytes, preceded by a header, a length and followed by a simple checksum.  

Packets must either contain multiple event messages (output) or a single command (input) or acknowledgement (output) message. 

Packets may be zero padded to fill up a predefined total packet length, so that the header and length can be sent before streaming the messages with minimal latency.

| Content             | Description                                                                     |
| :------------------ | :------------------------------------------------------------------------------ |
| 0x49 0x52 0x4F 0x4E | a magic header consisting of the ASCII characters 'I','R','O','N' in that order |
| uint16              | a big endian 16 bit byte count, up to 1024, of the following messages           |
| messages            | a sequence of messages as defined below                                         |
| uint16              | the sum over all message bytes as a big-endian 16 bit number                    |

Packet Format

(The checksum is a simple sum, which is not very resilient against common types of serial errors. consider replacing with a CRC16 in the future.)

#### Message Format

Event messages are in a format close to a CoaXPress Event Message or a CoaXPress Tagged Command or Acknowledgement Message for ease of forwarding encapsulated in a CoaXPress Event Packet or in a CoaXPress Tagged Command or Tagged Acknowledgement Packet.

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

Example big-endian encoding of four 8-, two 16-, one 32- or 64-bit values in Words.

####  Command Messages (input)

A packet may contain a single command message with the following layout:
| Word   | Content (hex) | Description                                            |
| :----- | :------------ | :----------------------------------------------------- |
| 0      | 05 05 05 05   | Control command indication – with tag.                 |
| 1      | 4x Tag        | To be repeated in reply (see below).                   |
| 2      | Cmd           | Bit 31..24: 0x00 Memory read, 0x01 Memory write        |
|        | Size          | Bit 23..0: Number of bytes N to read or write          |
| 3      | Addr          | 32 bit address of data to read or write                |
| 3..N+2 | Data          | for writes: payload data , zero padded to multiple of 4 |
| N+3    | CRC32         | 32 bit CRC calculated over data words 1 to N+2.        |

Control command message format  – with tag  -- cf CoaXPress Standard Version 2.1 p.61 Table 24.

The CRC is the one defined in CoaxPress Standard v 2.2 section 9.2.2.2. which is the Ethernet CRC with some additional byte and bit swapping (TODO).

The Address space for memory read/write commands is defined in the section 'Command Address Space' below


####  Acknowledge Messages (output)

A packet may contain a single acknowledge message with the following layout:

| Word   | Content (hex) | Description                                     |
| :----- | :------------ | :---------------------------------------------- |
| 0      | 06 06 06 06   | Control acknowledge indication – with tag.      |
| 1      | 4x Tag        | Tag, matching the command being acknowledged.   |
| 2      | 4x Code       | Acknowledgment code (repeated 4 times)          |
| 3      | Size          | Number of bytes N in payload                    |
| 4..N+3 | Data          | payload data , zero padded to multiple of 4      |
| N+4    | CRC32         | 32 bit CRC calculated over data words 1 to N+3. |

Acknowledgment message format – with tag  -- cf CoaXPress Standard Version 2.1 p.63 Table 26.


Acknowledgment codes:
| Code | Meaning                                                                                                         |
| ---- | --------------------------------------------------------------------------------------------------------------- |
| 0x00 | Command executed OK, reply data is appended (i.e. acknowledgment of read command).                              |
| 0x01 | Command executed OK, No reply data is appended (i.e. acknowledgment of write command).                          |
| 0x40 | Invalid address.                                                                                                |
| 0x41 | Invalid data for the address.                                                                                   |
| 0x42 | Invalid control operation code.                                                                                 |
| 0x43 | Write attempted to a read-only address.                                                                         |
| 0x44 | Read attempted from a write-only address.                                                                       |
| 0x45 | Size field too large – command message (write) or acknowledgment message (read) would exceed packet size limit. |
| 0x46 | Incorrect size received, message size is inconsistent with message size indication.                             |
| 0x47 | Malformed packet.                                                                                               |
| 0x80 | Failed CRC test in last received command. Other values are reserved for future use.                             |

For all acknowledgment codes other than 0x00 the Size, Data and CRC fields must be omitted.

Acknowledgment messages are sent in reply to a command.

Since 0x06060606 is not a valid Word 0 for the Event Messages defined below, the first word in a packet can be used
to determine if a packet contains a single 

#### Event Messages (output)

A packet may contain multiple event messages with the following layout:

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

Note: The BMInator does not emit events larger than 32 bytes.

Note: The Timestamp field is filled in by the BMInator microcontroller referenced to its internal clock, with no relation to the Host Unit's internal clock. Strictly speaking this is not conform the CoaXPress standard.

The BMInator generates messages with the following ID (Namespace + EventID) and contents

|     ID | Size | Word 0 (hex) | Freq. (Hz) | Name                       | Type      | Content                                                        |
| -----: | ---: | -----------: | ---------: | :------------------------- | :-------- | :------------------------------------------------------------- |
| 0x8003 |   20 |  00 14 80 03 |          1 | ID0                        | [2]uint32 | software revision:UUID[2]                                      |
| 0x8004 |   20 |  00 14 80 04 |          1 | ID1                        | [2]uint32 | microcontroller UUID[1]:UUID[0]                                |
| 0x8020 |   20 |  00 14 80 20 |          1 | BARO                       | [2]uint32 | BME280 temperature (milli Kelvin) and pressure (milli Pascal)  |
| 0x8021 |   20 |  00 14 80 21 |          1 | HUMID                      | [2]uint32 | BME280 humidity measurement                                    |
| 0x8022 |   20 |  00 14 80 22 |          1 | TEMP                       | [2]uint32 | BMI088 temperature, STM32L43x temperature (milli Kelvin)       |
| 0x8023 |   20 |  00 14 80 23 |        FPS | time pulse reference OPEN  | uint64    | Host Unit time pulse reference raise event, payload is counter |
| 0x8025 |   20 |  00 14 80 25 |        FPS | time pulse reference CLOSE | uint64    | Host Unit time pulse reference fall event, payload is counter  |
| 0x8032 |   20 |  00 14 80 32 |       1600 | ACCEL_3G                   | 4x int16  | BMI088 raw accelerometer reading x,y,z + 0x0000 padding        |
| 0x8033 |      |  00 14 80 33 |            | ACCEL_6G                   |           | see note                                                       |
| 0x8034 |      |  00 14 80 34 |            | ACCEL_12G                  |           | see note                                                       |
| 0x8035 |      |  00 14 80 35 |            | ACCEL_24G                  |           | see note                                                       |
| 0x8038 |   20 |  00 14 80 38 |       2000 | GYRO_125DEG_S              | 4x int16  | BMI088 raw gyroscope reading x,y,z + 0x0000 padding            |
| 0x8039 |      |  00 14 80 39 |            | GYRO_250DEG_S              |           | see note                                                       |
| 0x803a |      |  00 14 80 3a |            | GYRO_500DEG_S              |           | see note                                                       |
| 0x803b |      |  00 14 80 3b |            | GYRO_1000DEG_S             |           | see note                                                       |
| 0x803c |      |  00 14 80 3c |            | GYRO_2000DEG_S             |           | see note                                                       |

Event Message types. 

Note: the BMInator will send only one of the message types 0x032..0x35 and of 0x038..0x3c, depending on the current sensitiviy setting of its Gyroscope and Accelerometer components.  
The eventid encodes the full scale value of the int16 numbers as per the BMI085 datasheet.

Event types 0x23 and 0x25 have a timestamp recorded in the Event Message header as described above. They are sent at the Host Unit frame rate. Their payload is a continuously incremented counter.

The bandwidth will be dominated by the 3600Hz messages of 20 bytes each, or 72 kB/s, taking up 720 kBaud on the serial line, about or 80%. 

The two padding bytes in the ACCEL and GYRO messages may be set to zero or may be used for an error correction code on the payload, TBD(lvd).

The streaming event messages are all exactly 20 bytes in length to ease the framing and de-framing as described above. 

### Command Address Space

The Command/Acknowledge packets allow the host unit to control the bminator by reading and writing registers.
The BMInator defines the following address layout, within a prefix meant to deconflict the host unit's own address space.

| Address range      | r/w | Semantics                         |
| ------------------ | --- | --------------------------------- |
| 0x2300 xxxx        |     | Prefix defining a 64kb space      |
| prefix 0100 - 017f | r/o | BMI088 Gyro Register map          |
| prefix 0140        | rw  | BMI088 Gyro config register 0x40  |
| prefix 0141        | rw  | BMI088 Gyro range register 0x41   |
| prefix 0200 - 027f | r/o | BMI088 Accel Register map         |
| prefix 020F        | rw  | BMI088 Accel config register 0x0F |
| prefix 0210        | rw  | BMI088 Accel range register 0x10  |
| prefix 0400 - 047f | r/o | BME280 Register map               |
| prefix 0472        | rw  | BME280 ctrl hum register 0x72     |
| prefix 0474        | rw  | BME280 ctrl meas register 0x74    |
| prefix 0475        | rw  | BME280 config register 0x75       |
| prefix 1xxx        | ro  | TODO uC internal variables        |
| prefix 2xxx        | rw  | TODO Heater Temperature control   |

