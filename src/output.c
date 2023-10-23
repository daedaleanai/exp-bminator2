#include "output.h"

#include "binary.h"
#include "bmxspi.h"
#include "bmi08x.h"
#include "bme280.h"

uint32_t gyro_hdr = 0;   // set in main, from the gyro config
uint32_t accel_hdr = 0;  // same for accel
struct LinearisationParameters bmeParam;  // needed to decode humidity sensor values

// these are decoded in one message, but sent on in others.
static int32_t accel_temp_mk = -1; // last observed value from BMI088 accelerator temperature 
static int32_t bme_hume6 = -1;     // last observed value of BME280 humidity

int output(struct Msg *msg , struct SPIXmit *x) {

#define MSGTYPE(addr, tag) (((uint16_t)(addr) & 0xf) << 8) | ((uint8_t)(tag)&0x7f)
    switch (MSGTYPE(x->addr, x->tag)) {
    case MSGTYPE(GYRO, BMI08x_RATE_X_LSB):
        if (x->buf[9] & 0x80) {
            msg_append16(msg, 0);  // header len
            msg_append16(msg, gyro_hdr);  // header
            msg_append64(msg, x->ts);
            msg_append16(msg, decode_le_uint16(x->buf + 1));
            msg_append16(msg, decode_le_uint16(x->buf + 3));
            msg_append16(msg, decode_le_uint16(x->buf + 5));
            msg_append16(msg, 0);  // pad
            msg->buf[1] = msg->len;
            return 1;
        }
        return 0;

    case MSGTYPE(ACCEL, BMI08x_ACC_X_LSB):
        if (x->buf[13] & 0x80) {
            // uint32_t ts = decode_le_uint24(x->buf + 8);  innner timestamp of accelerator, not used
            accel_temp_mk = bmi08x_decode_temp_mdegc(x->buf + 18) + 273150;  // sent separately at 1Hz
            msg_append16(msg, 0);  // len
            msg_append16(msg, accel_hdr);  // header
            msg_append64(msg, x->ts);
            msg_append16(msg, decode_le_uint16(x->buf + 2));
            msg_append16(msg, decode_le_uint16(x->buf + 4));
            msg_append16(msg, decode_le_uint16(x->buf + 6));
            msg_append16(msg, 0);  // pad
            msg->buf[1] = msg->len;
            return 1;
        }
        return 0;

    case MSGTYPE(HUMID, BME280_DATA_REG):
        if (1) { // TODO: look for valid flag
            int32_t t_mdegc,  p_mpa;
            bme_decode(&bmeParam, x->buf, &t_mdegc, &p_mpa, &bme_hume6);
            msg_append16(msg, 0);  // len
            msg_append16(msg, EVENTID_BARO);  // header
            msg_append64(msg, x->ts);
            msg_append32(msg, t_mdegc + 273150); // convert to milliKelvin
            msg_append32(msg, p_mpa);
            msg->buf[1] = msg->len;
            return 1;
        }   
    }
#undef MSGTYPE

    return 0;
}
  


#if 0

TODO

    

            } else if ((ts = unlatch(&periodic_ts)) != 0) {
                msg_append16(msg, periodic_headers[periodic_cnt % 3]);  // header
                msg_append16(msg, 0x8014);  // header
                msg_append64(msg, ts);
                msg_append32(msg, periodic_values[periodic_cnt % 3][0]);
                msg_append32(msg, periodic_values[periodic_cnt % 3][1]);
                ++periodic_cnt;

            }

#endif