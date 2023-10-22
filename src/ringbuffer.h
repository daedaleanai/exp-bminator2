#pragma once

#include <stddef.h>  // for size_t
#include <stdint.h>  // for uintX_t

// A simple character fifo
struct Ringbuffer {
    uint8_t  buf[1 << 9];  // size must be power of two for efficiency in taking modulo index
    uint16_t head;         // writes happen here
    uint16_t tail;         // reads happen here
};

// invariant:
// as long as you only put_head() if !full(), and get_tail() if !empty(), tail <= head and head-tail < sizeof buf.

inline uint16_t ringbuffer_avail(const struct Ringbuffer *rb) { return rb->head - rb->tail; }                     // 0..size
inline uint16_t ringbuffer_free(const struct Ringbuffer *rb)  { return sizeof(rb->buf) - (rb->head - rb->tail); } // size  .. 0
inline int      ringbuffer_empty(const struct Ringbuffer *rb) { return rb->head == rb->tail; }
inline int      ringbuffer_full(const struct Ringbuffer *rb)  { return rb->head == rb->tail + sizeof rb->buf; }
inline void     ringbuffer_clear(struct Ringbuffer *rb)       { rb->head = rb->tail; } // may race with get_tail and put_head
inline void     ringbuffer_put_head(struct Ringbuffer *rb, uint8_t c)    { rb->buf[rb->head++ % (sizeof rb->buf)] = c; }
inline uint8_t  ringbuffer_get_tail(struct Ringbuffer *rb)               { return rb->buf[rb->tail++ % (sizeof rb->buf)]; }

