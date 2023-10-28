#pragma once
#include <stdint.h>

// little endian binary decoding

inline uint16_t decode_le_uint16(uint8_t *buf) {
    return ((uint16_t)(buf[0])) | (((uint16_t)(buf[1])) << 8);
}

inline uint32_t decode_le_uint24(uint8_t *buf) {
    return ((uint32_t)(buf[0])) | (((uint32_t)(buf[1])) << 8) | (((uint32_t)(buf[2])) << 16);
}

inline uint32_t decode_le_uint32(uint8_t *buf) {
    return ((uint32_t)(buf[0])) | (((uint32_t)(buf[1])) << 8) | (((uint32_t)(buf[2])) << 16) | (((uint32_t)(buf[3])) << 24);
}

inline uint64_t decode_le_uint64(uint8_t *buf) {
    return ((uint64_t)(buf[0])) | (((uint64_t)(buf[1])) << 8) | (((uint64_t)(buf[2])) << 16) | (((uint64_t)(buf[3])) << 24) 
         | (((uint64_t)(buf[4])) << 32) | (((uint64_t)(buf[5])) << 40) | (((uint64_t)(buf[6])) << 48) | (((uint64_t)(buf[7])) << 56);
}

// big endian binary decoding
inline uint16_t decode_be_uint16(uint8_t *buf) {
    return ((uint16_t)(buf[0]) << 8) | ((uint16_t)(buf[1]));
}

inline uint32_t decode_be_uint24(uint8_t *buf) {
    return (((uint32_t)(buf[0])) << 16) | (((uint32_t)(buf[1])) << 8) | ((uint32_t)(buf[2]));
}

inline uint32_t decode_be_uint32(uint8_t *buf) {
    return ((uint32_t)(buf[0]) << 24) | (((uint32_t)(buf[1])) << 16) | (((uint32_t)(buf[2])) << 8) | ((uint32_t)(buf[3]));
}

inline uint64_t decode_be_uint64(uint8_t *buf) {
    return ((uint64_t)(buf[0]) << 56) | (((uint64_t)(buf[1])) << 48) | (((uint64_t)(buf[2])) << 40) | (((uint64_t)(buf[3])) << 32) 
         | ((uint64_t)(buf[4]) << 24) | (((uint64_t)(buf[5])) << 16) | (((uint64_t)(buf[6])) << 8) | ((uint64_t)(buf[7]));
}

// little endian binary encoding
inline void encode_le_uint16(uint8_t *buf, uint16_t val) {
    *buf++ = val;
    *buf++ = val >> 8;
}

inline void encode_le_uint24(uint8_t *buf, uint32_t val) {
    *buf++ = val;
    *buf++ = val >> 8;
    *buf++ = val >> 16;
}

inline void encode_le_uint32(uint8_t *buf, uint32_t val) {
    *buf++ = val;
    *buf++ = val >> 8;
    *buf++ = val >> 16;
    *buf++ = val >> 24;
}

inline void encode_le_uint64(uint8_t *buf, uint64_t val) {
    *buf++ = val;
    *buf++ = val >> 8;
    *buf++ = val >> 16;
    *buf++ = val >> 24;
    *buf++ = val >> 32;
    *buf++ = val >> 40;
    *buf++ = val >> 48;
    *buf++ = val >> 56;
}

// big endian binary encoding
inline void encode_be_uint16(uint8_t *buf, uint16_t val) {
    *buf++ = val >> 8;
    *buf++ = val;
}

inline void encode_be_uint24(uint8_t *buf, uint32_t val) {
    *buf++ = val >> 16;
    *buf++ = val >> 8;
    *buf++ = val;
}

inline void encode_be_uint32(uint8_t *buf, uint32_t val) {
    *buf++ = val >> 24;
    *buf++ = val >> 16;
    *buf++ = val >> 8;
    *buf++ = val;
}

inline void encode_be_uint64(uint8_t *buf, uint64_t val) {
    *buf++ = val >> 56;
    *buf++ = val >> 48;
    *buf++ = val >> 40;
    *buf++ = val >> 32;
    *buf++ = val >> 24;
    *buf++ = val >> 16;
    *buf++ = val >> 8;
    *buf++ = val;
}
