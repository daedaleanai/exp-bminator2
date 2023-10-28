#pragma once 
#include <stddef.h>
#include <stdint.h>

#include "binary.h"

struct Msg {
    size_t len;
    uint8_t buf[20];
};

inline void msg_reset(struct Msg *msg) {
    msg->len = 0;
}

// binary encoding big endian numbers
inline void msg_append16(struct Msg *msg, uint16_t val) {
    encode_be_uint16(msg->buf+msg->len, val);
    msg->len += 2;
}

inline void msg_append32(struct Msg *msg, uint32_t val) {
    encode_be_uint32(msg->buf+msg->len, val);
    msg->len += 4;
}

inline void msg_append64(struct Msg *msg, uint64_t val) {
    encode_be_uint64(msg->buf+msg->len, val);
    msg->len += 8;
}

struct MsgQueue {
    volatile uint32_t head;
    volatile uint32_t tail;
    struct Msg elem[8];
};

#define NELEM(x) (sizeof(x) / sizeof(x[0]))
// return null if empty/full
inline struct Msg *msgq_tail(struct MsgQueue *q) { return (q->head == q->tail)                  ? NULL : &q->elem[q->tail % NELEM(q->elem)]; }
inline struct Msg *msgq_head(struct MsgQueue *q) { return (q->head == q->tail + NELEM(q->elem)) ? NULL : &q->elem[q->head % NELEM(q->elem)]; }
#undef NELEM

// call these exactly once for each non null returned value of msgq_head and _tail
inline void msgq_pop_tail(struct MsgQueue *q) { ++q->tail; }
inline void msgq_push_head(struct MsgQueue *q) { ++q->head; }
