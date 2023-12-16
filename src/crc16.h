#pragma once

#include <stdlib.h>
#include <stdint.h>

uint16_t crc16_update(uint16_t crc, const uint8_t *data, size_t data_len);
