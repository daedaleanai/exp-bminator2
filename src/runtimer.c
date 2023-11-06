#include "runtimer.h"

#include "cortex_m4.h"
#include "tprintf.h"
#include "clock.h"

#define printf tprintf

void rt_report(struct RunTimer* rt, uint64_t* lastreport) {
    uint64_t now = cycleCount();
    uint32_t dt = now - *lastreport;
//  printf("elapsed %ld us\n", dt/C_US);
    printf("-------------  cnt - period - cum - max\n");
    while (rt) {
        __disable_irq();
        uint32_t cum = rt->cum;// accumulated runtime
        uint32_t cnt = rt->cnt; // number of times stopped
        uint32_t max = rt->max; // max time
        rt->cum = 0;
        rt->cnt = 0;
        rt->max = 0;
        __enable_irq();

        if (cnt == 0) {
            printf("%12s\e[K\n", rt->name);
        } else {
            uint64_t period = dt / (C_US * cnt);
            printf("%12s % 5ld % 8lld % 5ld % 3ld\e[K\n", rt->name, cnt, period, cum/C_US, max/C_US);
        }
        rt = rt->next;
    }
    *lastreport = now;
}