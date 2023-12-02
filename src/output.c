#include "output.h"

#include "binary.h"
#include "bme280.h"
#include "bmi08x.h"
#include "bmxspi.h"

uint32_t					   gyro_hdr	 = 0;  // set in main, from the gyro config
uint32_t					   accel_hdr = 0;  // same for accel
struct LinearisationParameters bmeParam;	   // needed to decode humidity sensor values

// these are decoded in one message, but sent on in others.
static int32_t  accel_temp_mk = 0;	// last observed value from BMI088 accelerator temperature
static int32_t  bme_hume6     = 0;
static uint64_t bme_ts        = 0;

static uint8_t zeroes[4] = { 0,0,0,0 };

#define MSGTYPE(addr, tag) (((uint16_t)(addr)&0xf) << 8) | ((uint8_t)(tag)&0xff)

void output_bmx(struct Msg *msg, struct SPIXmit *x) {
	msg_reset(msg);

	if (x->tag & 0xff000000) {
		msg_append32(msg, bytex4(x->tag >> 16)); // tagtagtagtag
		uint8_t code = (x->tag & 0x80) ? 0 : 1; // read (0) or write (1)
		if (x->status != 0) {
			code += 0x48;  // read (48) or write (49) failed
		}
		msg_append32(msg, bytex4(code)); 
		if (x->tag & 0x80) { // read
			msg_append32(msg, x->len - 1); // number of bytes read 
		}
		msg_appendbuf(msg, x->buf, x->len - 1); 
		msg_appendbuf(msg, zeroes, (4 - (x->len - 1)%4)%4 ); 
		msg_append32(msg, 0x12345678); // crc32 
		return;
	}


	switch (MSGTYPE(x->addr, x->tag)) {
	case MSGTYPE(GYRO, 0x80 | BMI08x_RATE_X_LSB):
		msg_append16(msg, 0);		  // header len
		msg_append16(msg, gyro_hdr);  // header
		msg_append64(msg, x->ts);
		if (x->buf[9] & 0x80) {
			msg_append16(msg, decode_le_uint16(x->buf + 1));
			msg_append16(msg, decode_le_uint16(x->buf + 3));
			msg_append16(msg, decode_le_uint16(x->buf + 5));
			msg_append16(msg, 0);  // pad
		} else {
			msg_append64(msg, -1);
		}
		msg->buf[1] = msg->len;
		return;

	case MSGTYPE(ACCEL, 0x80 | BMI08x_ACC_X_LSB):
		// uint32_t ts = decode_le_uint24(x->buf + 8);  innner timestamp of accelerator, not used
		// accel_temp is sent separately along with the ADC temperature
		msg_append16(msg, 0);		  // len
		msg_append16(msg, accel_hdr); // header
		msg_append64(msg, x->ts);
		if (x->buf[13] & 0x80) {
			accel_temp_mk = bmi08x_decode_temp_mdegc(x->buf + 18) /*+ 273150*/;	 

			msg_append16(msg, decode_le_uint16(x->buf + 2));
			msg_append16(msg, decode_le_uint16(x->buf + 4));
			msg_append16(msg, decode_le_uint16(x->buf + 6));
			msg_append16(msg, 0);  // pad
		} else {			
			msg_append64(msg, -1);
		}
		msg->buf[1] = msg->len;
		return;

	case MSGTYPE(HUMID, 0x80 | BME280_DATA_REG):
		msg_append16(msg, 0);			  // len
		msg_append16(msg, EVENTID_BARO);  // header
		msg_append64(msg, x->ts);
		{
			int32_t t_mdegc, p_mpa;
			bme_decode(&bmeParam, x->buf+1, &t_mdegc, &p_mpa, &bme_hume6);
			bme_ts = x->ts;
			msg_append32(msg, t_mdegc);  // + 273150 to convert to milliKelvin
			msg_append32(msg, p_mpa);
		}
		msg->buf[1] = msg->len;
		return;
	}

	// in all other cases, return a generic 'spi result' message
	msg_append16(msg, 0);			  // len
	msg_append16(msg, EVENTID_GENERICSPI+(x->tag & 0xff));  // header
	msg_appendbuf(msg, x->buf, x->len);
	msg->buf[1] = msg->len;

	return;
}

int output_humidity(struct MsgQueue *msgq) {
	struct Msg* msg = msgq_head(msgq);
	if (!msg) {
		return 1;
	}

	msg_reset(msg);
	msg_append16(msg, 0);			  // len
	msg_append16(msg, EVENTID_HUMID);  // header
	msg_append64(msg, bme_ts);
	msg_append32(msg, bme_hume6);  
	msg_append32(msg, 0);  // pad
	msg->buf[1] = msg->len;
	msgq_push_head(msgq);

	return 0;
}


// calibration parameters defined in .ld file
extern uint16_t TS_CAL1, TS_CAL2, VREFINT;

int output_internaltemperature(struct MsgQueue *msgq, uint64_t ts, uint16_t vref_adc_val, uint16_t ts_adc_val) {
	struct Msg* msg = msgq_head(msgq);
	if (!msg) {
		return 1;
	}

	msg_reset(msg);
	msg_append16(msg, 0);			  // len
	msg_append16(msg, EVENTID_TEMP);  // header
	msg_append64(msg, ts);
	if ((vref_adc_val > 0) && (TS_CAL2 != TS_CAL1)) {
		// TODO(lvd) this appears to be wrong, either with or without correcting for Vdd
		int64_t x = ts_adc_val;
		x *= VREFINT;
		x /= vref_adc_val;
//		int64_t temp = ((x - TS_CAL1) * (130000+273150) - (x - TS_CAL2) * (30000+273150) ) / (TS_CAL2 - TS_CAL1);
		int64_t temp = ((x - TS_CAL1) * (130000) - (x - TS_CAL2) * (30000) ) / (TS_CAL2 - TS_CAL1);
		msg_append32(msg, temp);  
	} else {
		msg_append32(msg, -1);  
	}
	msg_append32(msg, accel_temp_mk);  // old one but who cares
	msg->buf[1] = msg->len;
	msgq_push_head(msgq);
	return 0;
}


int output_shutter(struct MsgQueue *msgq, uint16_t hdr, uint64_t ts, uint64_t counter) {
	struct Msg *msg = msgq_head(msgq);
	if (msg == NULL) {
		return 1;
	}

	msg_reset(msg);
	msg_append16(msg, 0);	 // len
	msg_append16(msg, hdr);	 // header
	msg_append64(msg, ts);
	msg_append64(msg, counter);
	msg->buf[1] = msg->len;
	msgq_push_head(msgq);
	return 0;
}

int output_periodic(struct MsgQueue *msgq, uint16_t hdr, uint64_t ts, uint32_t v1, uint32_t v2) {
	struct Msg *msg = msgq_head(msgq);
	if (msg == NULL) {
		return 1;
	}
	msg_reset(msg);
	msg_append16(msg, 0);	 // len
	msg_append16(msg, hdr);	 // header
	msg_append64(msg, ts);
	msg_append32(msg, v1);
	msg_append32(msg, v2);
	msg->buf[1] = msg->len;
	msgq_push_head(msgq);
	return 0;
}