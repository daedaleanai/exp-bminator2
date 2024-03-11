#pragma once
#include "msgq.h"
#include "spi.h"

//  Message IDs
enum {
	EVENTID_ID0			  = 0x8003,	 // [2]uint32 appinfo.revision, serial number [2]
	EVENTID_ID1			  = 0x8004,	 // [2]uint32 Serial number[1][0]
	EVENTID_ADC           = 0x8010,  // [4]uint16 4 raw adc measurements: vref, thermistor, current, internal temp
	EVENTID_BARO		  = 0x8020,	 // [2]uint32 temperature[milliK] pressure[milliPa] bme280
	EVENTID_HUMID		  = 0x8021,	 // humidity bme280 [TODO]
	EVENTID_TEMP		  = 0x8022,	 // [2]uint32 temperature[milliK] (bmi085 accellerometer, stm32 microcontroller)
	EVENTID_SHUTTER_OPEN  = 0x8023,	 // uint64 counter
	EVENTID_SHUTTER_CLOSE = 0x8025,	 // uint64 counter

	// [4]int16 xyz_ (i.e. padded to 8 bytes)
	EVENTID_ACCEL_3G	   = 0x8032,  // RANGE_2G  = 0x00,  bmi088: 3g
	EVENTID_ACCEL_6G	   = 0x8033,  // RANGE_4G  = 0x01,  bmi088: 6g
	EVENTID_ACCEL_9G	   = 0x8034,  // RANGE_8G  = 0x02,          12g
	EVENTID_ACCEL_24G	   = 0x8035,  // RANGE_16G = 0x03,         24g
	EVENTID_GYRO_125DEG_S  = 0x8038,  // BMI085_GYRO_RANGE_125DEG_S  = 0x04,
	EVENTID_GYRO_250DEG_S  = 0x8039,  // BMI085_GYRO_RANGE_250DEG_S  = 0x03,
	EVENTID_GYRO_500DEG_S  = 0x803a,  // BMI085_GYRO_RANGE_500DEG_S  = 0x02,
	EVENTID_GYRO_1000DEG_S = 0x803b,  // BMI085_GYRO_RANGE_1000DEG_S = 0x01,
	EVENTID_GYRO_2000DEG_S = 0x803c,  // BMI085_GYRO_RANGE_2000DEG_S = 0x00,

	EVENTID_GENERICSPI = 0x8100,  // + read/writereg, up to 16 bytes of data
};

// these are set in main
extern uint32_t						  gyro_hdr;	  // which EVENTID_GYRO_xxxDEG_S currently active
extern uint32_t						  accel_hdr;  // which EVENTID_EVENTID_ACCEL_2G currently active
extern struct LinearisationParameters bmeParam;

// convert the SPI xmit messages from communicating with the BMI and BME
// to our output format.
void output_bmx(struct Msg *msg, struct SPIXmit *x);

int output_humidity(struct MsgQueue *msgq);
// this one sends along the last accel_temp as well
int output_internaltemperature(struct MsgQueue *msgq, uint64_t ts, uint16_t vref_adc_val, uint16_t ts_adc_val);
int output_shutter(struct MsgQueue *msgq, uint16_t hdr, uint64_t ts, uint64_t counter);
int output_periodic(struct MsgQueue *msgq, uint16_t hdr, uint64_t ts, uint32_t v1, uint32_t v2);
int output_adc(struct MsgQueue *msgq, uint64_t ts, uint16_t v0, uint16_t v1, uint16_t v2, uint16_t v3 );