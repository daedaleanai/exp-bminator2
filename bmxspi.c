#include "stm32l4xx.h"

#include "bmxspi.h"

#include "bmi08x.h"
#include "bme280.h"
#include "clock.h"
#include "tprintf.h"
#include "spi.h"

#define printf tprintf

static inline uint16_t decode_le_uint16(uint8_t *buf) {
    uint16_t r = *buf++;
    r |= ((uint16_t)(*buf++)) << 8;
    return r;
}

static inline uint32_t decode_le_uint24(uint8_t *buf) {
    uint32_t r = *buf++;
    r |= ((uint32_t)(*buf++)) << 8;
    r |= ((uint32_t)(*buf++)) << 16;
    return r;
}

static inline uint32_t decode_le_uint32(uint8_t *buf) {
    uint32_t r = *buf++;
    r |= ((uint32_t)(*buf++)) << 8;
    r |= ((uint32_t)(*buf++)) << 16;
    r |= ((uint32_t)(*buf++)) << 24;
    return r;
}

// blocking write single register
uint16_t bmx_writereg(struct SPIQ *q, enum BMXFunction bf, uint8_t reg, uint8_t val) {
    struct SPIXmit *x = spiq_head(q);
    if (!x) {
        return 0xffff;
    }
    uint8_t buf[2] = {reg, val};
    x->addr = bf;
    x->len = 2;
    x->buf = buf;
    spiq_enq_head(q);
    spi_wait(q);
    x = spiq_tail(q);
    if (!x) {
        return 0xfffe;
    }
    uint16_t r = x->status;
    spiq_deq_tail(q);
    return r;
}

// blocking read single register
uint16_t bmx_readreg(struct SPIQ *q, enum BMXFunction bf, uint8_t reg, uint8_t *val) {
    struct SPIXmit *x = spiq_head(q);
    if (!x) {
        return 0xffff;
    }
    reg |= 0x80;  // flag for reading
    uint8_t buf[3] = {reg, 0, 0};
    x->addr = bf;
    x->len = (bf == GYRO) ? 2 : 3;  // accel has 1 extra byte of garbage after first
    x->buf = buf;
    spiq_enq_head(q);
    spi_wait(q);
    x = spiq_tail(q);
    if (!x) {
        return 0xfffe;
    }
    *val = buf[x->len - 1];
    uint16_t r = x->status;
    spiq_deq_tail(q);
    return r;
}

// Cf. data sheet accel power on section.
uint16_t bmi_accel_poweron(struct SPIQ *q) {
    uint8_t dum = 0;
    uint16_t r = bmx_readreg(q, ACCEL, BMI08x_ACC_CHIP_ID, &dum);
    if (r != 0) {
        return r;
    }
    r = bmx_writereg(q, ACCEL, BMI08x_ACC_PWR_CTRL, 0x04);
    if (r != 0) {
        return r;
    }
    delay(50000);
    return bmx_writereg(q, ACCEL, BMI08x_ACC_PWR_CONF, 0x0);
}

static uint16_t bmi_read_accel(struct SPIQ *q, int16_t xyz[3]) {
    struct SPIXmit *x = spiq_head(q);
    if (!x) {
        return 0xffff;
    }
    uint8_t buf[8] = {0x80 | BMI08x_ACC_X_LSB, 0, 0, 0, 0, 0, 0, 0};
    x->addr = ACCEL;
    x->len = sizeof buf;
    x->buf = buf;
    spiq_enq_head(q);
    spi_wait(q);
    x = spiq_tail(q);
    if (!x) {
        return 0xfffe;
    }

    uint16_t r = x ->status;
    if (r == 0) {
        xyz[0] = decode_le_uint16(buf + 2);
        xyz[1] = decode_le_uint16(buf + 4);
        xyz[2] = decode_le_uint16(buf + 6);
    }
    spiq_deq_tail(q);
    return r;
}

int bmi088_self_test(struct SPIQ *q) {
    uint8_t chip_id = 0, err_code = 0;
    uint8_t err = 0;

    uint16_t r = bmx_readreg(q, GYRO, BMI08x_GYRO_CHIP_ID, &chip_id);
    if ((r != 0) || (chip_id != 0x0f)) {
        printf("Gyro chip_id %i (result %x)\n", chip_id, r);
        ++err;
    }

    printf("Starting gyro self test.\n");
    bmx_writereg(q, GYRO, BMI08x_GYRO_SELF_TEST, 0x01);  // trigger self test in gyro

    r = bmx_readreg(q, ACCEL, BMI08x_ACC_CHIP_ID, &chip_id);
    if (r == 0) {
        r = bmx_readreg(q, ACCEL, BMI08x_ACC_ERR_REG, &err_code);
    }
    if ((r != 0) || (chip_id != 0x1f) || (err_code != 0)) {
        printf("Accel chip_id %i error code %i (result %x)\n", chip_id, err_code, r);
        ++err;
    }

    printf("Starting accel self test.\n");

    // Note: the manual states an incorrect value for CONF
    bmx_writereg(q, ACCEL, BMI08x_ACC_RANGE, BMI085_ACC_RANGE_16G);  // ±16g range: +2^15 ~ 16G -> 1G = 2^11
    bmx_writereg(q, ACCEL, BMI08x_ACC_CONF,  BMI08x_ACC_CONF_1600HZ);  // ODR=1.6kHz, continuous sampling mode, “normal mode” (norm_avg4)
    delay(2500);  // wait > 2ms

    printf("  neutral");
    int16_t vals[3] = {0, 0, 0};
    r = bmi_read_accel(q, vals);
    if (r != 0) {
        printf(" spi error: %x", r);
        ++err;
    }
    printf(" xyz: %i %i %i\n", vals[0], vals[1], vals[2]);

    printf(" positive");
    bmx_writereg(q, ACCEL, BMI08x_ACC_SELF_TEST, 0x0D);  // Enable the positive self-test polarity
    delay(100000);  // Wait for > 50ms

    r = bmi_read_accel(q, vals);
    if (r != 0) {
        printf(" spi error: %x", r);
        ++err;
    }
    printf(" xyz: %i %i %i\n", vals[0], vals[1], vals[2]);

    printf(" negative");
    bmx_writereg(q, ACCEL, BMI08x_ACC_SELF_TEST, 0x09);  // Enable the negative self-test polarity
    delay(100000);  // Wait for > 50ms
    int16_t vals2[3] = {0, 0, 0};
    r = bmi_read_accel(q, vals2);
    if (r != 0) {
        printf(" spi error: %x", r);
        ++err;
    }
    printf(" xyz: %i %i %i\n", vals2[0], vals2[1], vals2[2]);
    bmx_writereg(q, ACCEL, BMI08x_ACC_SELF_TEST, 0x0);  // disable self-test

    if ((int32_t)vals[0] - (int32_t)vals2[0] < 2048) {
        printf("X-axis FAIL\n");
        ++err;
    }
    if ((int32_t)vals[1] - (int32_t)vals2[1] < 2048) {
        printf("Y-axis FAIL\n");
        ++err;
    }
    if ((int32_t)vals[2] - (int32_t)vals2[2] < 1024) {
        printf("Z-axis FAIL\n");
        ++err;
    }

    r = bmx_readreg(q, GYRO, BMI08x_GYRO_SELF_TEST, &err_code);
    if ((err_code & 0x16) != 0x12) {
        printf("Gyro self test FAIL register: %x (result %x)\n", err_code, r);
        ++err;
    } else {
        printf("Gyro self test OK.\n");
    }

    r = bmx_writereg(q, GYRO, BMI08x_GYRO_SOFTRESET, 0xB6);
    if (r) {
        printf("gyro reset spi error: %x\n", r);
        ++err;
    }

    r = bmx_writereg(q, ACCEL, BMI08x_ACC_SOFTRESET, 0xB6);
    if (r) {
        printf("acc reset spi error: %x\n", r);
        ++err;
    }
    delay(50000);

    printf("BMI08x self test completed with %i errors.\n", err);
    return err;
}

int bme280_self_test(struct SPIQ *q, struct LinearisationParameters *bmeParam) {
    uint16_t r = bmx_writereg(q, NONE, BME280_REG_RESET, BME280_RESET);
    if (r != 0) {
        printf("baro reset spi error: %x\n", r);
        return -1;
    }
    delay(20000);

    uint8_t chip_id = 0;
    r = bmx_readreg(q, NONE, BME280_REG_ID, &chip_id);

    if ((r != 0) || (chip_id != 0x60)) {
        printf("Baro chip_id %i (result %x)\n", chip_id, r);
        return -1;
    }

    (void)bmeParam;
#if 0
    struct SPIXmit *x = spiq_head(q);
    if (!x) {
        return -1;
    }
    uint8_t buf[BME280_CALIBTP_LEN+1] = {0x80 | BME280_CALIBTP_REG, 0};
    
    x->addr = HUMID;
    x->len = sizeof buf;
    x->buf = buf;
    spiq_enq_head(q);
    spi_wait(q);
    x = spiq_tail(q);
    if (!x) {
        return -1;
    }

    r = x ->status;
    printf("BCLBR:");
    if (r != 0) {
        spiq_deq_tail(q);
        printf(" ERROR %x:\n", r);
        return -1;
    } else {
        for (int i = 2; i < sizeof buf; ++i) {
            printf(" %i", buf[i]);
        }
        printf("\n");
        bme_decodeLinearisationParameters(bmeParam, buf + 2);
        spiq_deq_tail(q);

        for (int i = 1; i < 4; ++i) {
            printf("\tT[%i]=%li\n", i, bmeParam->T[i]);
        }
        for (int i = 1; i < 10; ++i) {
            printf("\tP[%i]=%li\n", i, bmeParam->P[i]);
        }
    }

    // TODO Humid calibration
#endif

    return 0;
}

uint16_t bmx_config(struct SPIQ *q, enum BMXFunction bf, struct bmx_config_t *configs) {
    for (; configs->reg != 0xFF; ++configs) {
        uint16_t r = bmx_writereg(q, bf, configs->reg, configs->val);
        if (r != 0) {
            return r;
        }
    }
    return 0;
}
