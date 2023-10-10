#pragma once

// Constants for the BMI085 and BMI088
// cf datasheet BST-BMI088-DS001-13 and BST-BMI085-DS001-13

enum {
    // Accelerometer registers
    BMI08x_ACC_CHIP_ID = 0x0,
    BMI08x_ACC_ERR_REG = 0x2,
    BMI08x_ACC_STATUS = 0x3,
    BMI08x_ACC_X_LSB = 0x12,
    BMI08x_ACC_X_MSB = 0x13,
    BMI08x_ACC_Y_LSB = 0x14,
    BMI08x_ACC_Y_MSB = 0x15,
    BMI08x_ACC_Z_LSB = 0x16,
    BMI08x_ACC_Z_MSB = 0x17,
    BMI08x_SENSORTIME_0 = 0x18,
    BMI08x_SENSORTIME_1 = 0x19,
    BMI08x_SENSORTIME_2 = 0x1A,
    BMI08x_ACC_INT_STAT_1 = 0x1D,
    BMI08x_TEMP_MSB = 0x22,
    BMI08x_TEMP_LSB = 0x23,
    BMI08x_ACC_CONF = 0x40,
    BMI08x_ACC_RANGE = 0x41,
    BMI08x_INT1_IO_CONF = 0x53,  // table says _CTRL
    BMI08x_INT2_IO_CONF = 0x54,  // table says _CTRL
    BMI08x_INT1_INT2_MAP_DATA = 0x58,  // table says _INT_MAP_DATA
    BMI08x_ACC_SELF_TEST = 0x6D,
    BMI08x_ACC_PWR_CONF = 0x7C,
    BMI08x_ACC_PWR_CTRL = 0x7D,
    BMI08x_ACC_SOFTRESET = 0x7E,

    // Configuration values
    // bandwidth = sample rate / 2.5 (normal)
    BMI08x_ACC_CONF_1600HZ = 0xAC,
    BMI08x_ACC_CONF_800HZ = 0xAB,
    BMI08x_ACC_CONF_400HZ = 0xAA,
    BMI08x_ACC_CONF_200HZ = 0xA9,
    BMI08x_ACC_CONF_100HZ = 0xA8,
    BMI08x_ACC_CONF_50HZ = 0xA7,
    BMI08x_ACC_CONF_25HZ = 0xA6,
    BMI08x_ACC_CONF_12_5HZ = 0xA5,

    // bandwidth = sample rate / 5 (2x oversampling)
    BMI08x_ACC_CONF_1600HZ_OSR2 = 0x9C,
    BMI08x_ACC_CONF_800HZ_OSR2 = 0x9B,
    BMI08x_ACC_CONF_400HZ_OSR2 = 0x9A,
    BMI08x_ACC_CONF_200HZ_OSR2 = 0x99,
    BMI08x_ACC_CONF_100HZ_OSR2 = 0x98,
    BMI08x_ACC_CONF_50HZ_OSR2 = 0x97,
    BMI08x_ACC_CONF_25HZ_OSR2 = 0x96,
    BMI08x_ACC_CONF_12_5HZ_OSR2 = 0x95,

    // bandwidth = sample rate / 10 (4x oversampling)
    BMI08x_ACC_CONF_1600HZ_OSR4 = 0x8C,
    BMI08x_ACC_CONF_800HZ_OSR4 = 0x8B,
    BMI08x_ACC_CONF_400HZ_OSR4 = 0x8A,
    BMI08x_ACC_CONF_200HZ_OSR4 = 0x89,
    BMI08x_ACC_CONF_100HZ_OSR4 = 0x88,
    BMI08x_ACC_CONF_50HZ_OSR4 = 0x87,
    BMI08x_ACC_CONF_25HZ_OSR4 = 0x86,
    BMI08x_ACC_CONF_12_5HZ_OSR4 = 0x85,

    // These ranges are the only difference between the BMI085 and the BMI08x
    BMI088_ACC_RANGE_24G = 0x03,
    BMI088_ACC_RANGE_12G = 0x02,
    BMI088_ACC_RANGE_6G = 0x01,
    BMI088_ACC_RANGE_3G = 0x00,

    BMI085_ACC_RANGE_16G = 0x03,
    BMI085_ACC_RANGE_8G = 0x02,
    BMI085_ACC_RANGE_4G = 0x01,
    BMI085_ACC_RANGE_2G = 0x00,

    // Gyroscope registers
    BMI08x_GYRO_CHIP_ID = 0x0,
    BMI08x_RATE_X_LSB = 0x2,
    BMI08x_RATE_X_MSB = 0x3,
    BMI08x_RATE_Y_LSB = 0x4,
    BMI08x_RATE_Y_MSB = 0x5,
    BMI08x_RATE_Z_LSB = 0x6,
    BMI08x_RATE_Z_MSB = 0x7,
    BMI08x_GYRO_INT_STAT_1 = 0xA,
    BMI08x_GYRO_RANGE = 0xF,
    BMI08x_GYRO_BANDWIDTH = 0x10,
    BMI08x_GYRO_LPM1 = 0x11,
    BMI08x_GYRO_SOFTRESET = 0x14,
    BMI08x_GYRO_INT_CTRL = 0x15,
    BMI08x_INT3_INT4_IO_CONF = 0x16,
    BMI08x_INT3_INT4_IO_MAP = 0x18,
    BMI08x_GYRO_SELF_TEST = 0x3C,

    // Configuration Values
    BMI08x_GYRO_RANGE_2000DEG_S = 0x00,
    BMI08x_GYRO_RANGE_1000DEG_S = 0x01,
    BMI08x_GYRO_RANGE_500DEG_S = 0x02,
    BMI08x_GYRO_RANGE_250DEG_S = 0x03,
    BMI08x_GYRO_RANGE_125DEG_S = 0x04,

    BMI08x_GYRO_BANDWIDTH_2000_532HZ = 0x80,
    BMI08x_GYRO_BANDWIDTH_2000_230HZ = 0x81,
    BMI08x_GYRO_BANDWIDTH_1000_116HZ = 0x82,
    BMI08x_GYRO_BANDWIDTH_400_47HZ = 0x83,
    BMI08x_GYRO_BANDWIDTH_200_23HZ = 0x84,
    BMI08x_GYRO_BANDWIDTH_100_12HZ = 0x85,
    BMI08x_GYRO_BANDWIDTH_200_64HZ = 0x86,
    BMI08x_GYRO_BANDWIDTH_100_32HZ = 0x87,

};

// decode temperature in millidegree Celsius according to 5.3.7 Register 0x22 – 0x23: Temperature sensor data
inline int32_t bmi08x_decode_temp_mdegc(uint8_t *buf) {
    int32_t t = ((((uint16_t)buf[0]) << 3) | (buf[1] >> 5));
    if (t >= (1 << 10))
        t -= 1 << 11;
    t += 23 << 3;
    t *= 125;
    return t;
}
