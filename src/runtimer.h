#pragma once

#include <stdint.h>

// a runtimer keeps track of frequency, duty cyle, max and min run times
struct RunTimer {
    const char* name;
    uint64_t start; // last started
    uint32_t cum; // accumulated runtime
    uint32_t cnt; // number of times stopped
    uint32_t max; // max time
    struct RunTimer* next; 
};

inline void rt_start(struct RunTimer* rt, uint64_t now) { rt->start = now; }

inline void rt_stop(struct RunTimer* rt, uint64_t now) {
    uint32_t d = now - rt->start; 
    rt->cum += d;
    rt->cnt++;
    if (rt->max < d) {
        rt->max = d;
    } 
}

void rt_report(struct RunTimer* first, uint64_t* lastreport);