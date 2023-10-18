#pragma once
#include "msgq.h"
#include "spi.h"

/*
  Message IDs
*/
enum {
    EVENTID_ID0 = 0x003,  // [2]uint32 appinfo.revision, serial number [2]
    EVENTID_ID1 = 0x004,  // [2]uint32 Serial number[1][0]
    EVENTID_BARO = 0x020,  // [2]uint32 temperature[milliK] pressure[milliPa] bme280
    EVENTID_HUMID = 0x021,  // humidity bme280
    EVENTID_TEMP = 0x022,  // [2]uint32 temperature[milliK] (bmi085 accellerometer, stm32 microcontroller)
    EVENTID_SHUTTER_OPEN = 0x023,  // uint64 prev ts
    EVENTID_SHUTTER_CLOSE = 0x025,  // uint64 prev ts

    // [4]int16 xyz_ (i.e. padded to 8 bytes)
    EVENTID_ACCEL_2G = 0x032,  // RANGE_2G  = 0x00,
    EVENTID_ACCEL_4G = 0x033,  // RANGE_4G  = 0x01,
    EVENTID_ACCEL_8G = 0x034,  // RANGE_8G  = 0x02,
    EVENTID_ACCEL_16G = 0x035,  // RANGE_16G = 0x03,
    EVENTID_GYRO_125DEG_S = 0x038,  // BMI085_GYRO_RANGE_125DEG_S  = 0x04,
    EVENTID_GYRO_250DEG_S = 0x039,  // BMI085_GYRO_RANGE_250DEG_S  = 0x03,
    EVENTID_GYRO_500DEG_S = 0x03a,  // BMI085_GYRO_RANGE_500DEG_S  = 0x02,
    EVENTID_GYRO_1000DEG_S = 0x03b,  // BMI085_GYRO_RANGE_1000DEG_S = 0x01,
    EVENTID_GYRO_2000DEG_S = 0x03c,  // BMI085_GYRO_RANGE_2000DEG_S = 0x00,
};

extern uint32_t gyro_hdr;
extern uint32_t accel_hdr;
extern struct LinearisationParameters bmeParam;

int output(struct Msg *msg , struct SPIXmit *x);
