#pragma once
#include <stddef.h>

#include "bme280.h"
#include "spi.h"

// The BMI08x is basically 2 devices sharing an SPI, each have a separate CSB and INT line.
enum BMXFunction { NONE = 0, GYRO, ACCEL, HUMID };

struct bmx_config_t {
    uint8_t reg, val;
};

uint16_t bmx_config(struct SPIQ *spiq, enum BMXFunction bf, struct bmx_config_t *configs);

// Initialisation functions
uint16_t bmi_accel_poweron(struct SPIQ *spiq);
int bmi088_self_test(struct SPIQ *spiq);
int bme280_self_test(struct SPIQ *spiq, struct LinearisationParameters *bmeParam);
