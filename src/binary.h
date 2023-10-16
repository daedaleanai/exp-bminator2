#pragma once
#include <stdint.h>
// little  endian binary decoding, as needed

inline uint16_t decode_le_uint16(uint8_t *buf) {
    uint16_t r = *buf++;
    r |= ((uint16_t)(*buf++)) << 8;
    return r;
}

inline uint32_t decode_le_uint24(uint8_t *buf) {
    uint32_t r = *buf++;
    r |= ((uint32_t)(*buf++)) << 8;
    r |= ((uint32_t)(*buf++)) << 16;
    return r;
}

inline uint32_t decode_le_uint32(uint8_t *buf) {
    uint32_t r = *buf++;
    r |= ((uint32_t)(*buf++)) << 8;
    r |= ((uint32_t)(*buf++)) << 16;
    r |= ((uint32_t)(*buf++)) << 24;
    return r;
}