#include "ringbuffer.h"

// definitions are required in one compilation unit
// _full tests for free < 2 instead of == 0 so that there's room for the final '\n' or 0x7e
extern inline uint16_t ringbuffer_avail(const struct Ringbuffer *rb);
extern inline uint16_t ringbuffer_free(const struct Ringbuffer *rb);
extern inline int      ringbuffer_empty(const struct Ringbuffer *rb);
extern inline int      ringbuffer_full(const struct Ringbuffer *rb);
extern inline void     ringbuffer_clear(struct Ringbuffer *rb);
extern inline void     ringbuffer_put_head(struct Ringbuffer *rb, uint8_t c);
extern inline uint8_t  ringbuffer_get_tail(struct Ringbuffer *rb);
