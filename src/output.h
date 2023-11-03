#pragma once
#include "msgq.h"
#include "spi.h"

/*
  Message IDs
*/
enum {
    EVENTID_ID0 = 0x8003,  // [2]uint32 appinfo.revision, serial number [2]
    EVENTID_ID1 = 0x8004,  // [2]uint32 Serial number[1][0]
    EVENTID_BARO = 0x8020,  // [2]uint32 temperature[milliK] pressure[milliPa] bme280
    EVENTID_HUMID = 0x8021,  // humidity bme280 [TODO]
    EVENTID_TEMP = 0x8022,  // [2]uint32 temperature[milliK] (bmi085 accellerometer, stm32 microcontroller)
    EVENTID_SHUTTER_OPEN = 0x8023,  // uint64 counter
    EVENTID_SHUTTER_CLOSE = 0x8025,  // uint64 counter

    // [4]int16 xyz_ (i.e. padded to 8 bytes)
    EVENTID_ACCEL_2G = 0x8032,  // RANGE_2G  = 0x00,  bmi088: 3g
    EVENTID_ACCEL_4G = 0x8033,  // RANGE_4G  = 0x01,  bmi088: 6g
    EVENTID_ACCEL_8G = 0x8034,  // RANGE_8G  = 0x02,          12g
    EVENTID_ACCEL_16G = 0x8035,  // RANGE_16G = 0x03,         24g
    EVENTID_GYRO_125DEG_S = 0x8038,  // BMI085_GYRO_RANGE_125DEG_S  = 0x04,
    EVENTID_GYRO_250DEG_S = 0x8039,  // BMI085_GYRO_RANGE_250DEG_S  = 0x03,
    EVENTID_GYRO_500DEG_S = 0x803a,  // BMI085_GYRO_RANGE_500DEG_S  = 0x02,
    EVENTID_GYRO_1000DEG_S = 0x803b,  // BMI085_GYRO_RANGE_1000DEG_S = 0x01,
    EVENTID_GYRO_2000DEG_S = 0x803c,  // BMI085_GYRO_RANGE_2000DEG_S = 0x00,
};

extern uint32_t gyro_hdr;
extern uint32_t accel_hdr;
extern struct LinearisationParameters bmeParam;

// convert the SPI xmit messages from communicating with the BMI and BME
// to our output format.  Return 1 if a valid message was constructed.
int output_bmx(struct Msg *msg, struct SPIXmit *x);

int output_shutter(struct Msg *msg, uint16_t hdr, uint64_t ts, uint64_t counter);

int output_periodic(struct Msg *msg, uint16_t hdr, uint64_t ts, uint32_t v1, uint32_t v2);