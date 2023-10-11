#pragma once
#include <stddef.h>

#include "bme280.h"
#include "spi.h"

enum BMXFunction { NONE = 0, GYRO, ACCEL, HUMID, CURRSENSE };

struct bmx_config_t {
    uint8_t reg, val;
};

uint16_t bmx_config(struct SPIQ *spiq, enum BMXFunction bf, struct bmx_config_t *configs);

// Initialisation functions
uint16_t bmi_accel_poweron(struct SPIQ *spiq);
int bmi088_self_test(struct SPIQ *spiq);
int bme280_self_test(struct SPIQ *spiq, struct LinearisationParameters *bmeParam);

uint16_t bmx_readreg(struct SPIQ *q, enum BMXFunction bf, uint8_t reg, uint8_t *val);
uint16_t bmx_writereg(struct SPIQ *q, enum BMXFunction bf, uint8_t reg, uint8_t val);
