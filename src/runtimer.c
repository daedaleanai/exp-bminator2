#include "runtimer.h"

#include "cortex_m4.h"
#include "tprintf.h"
#include "clock.h"

#define printf tprintf

void rt_report(struct RunTimer* rt, uint64_t* lastreport) {
    uint64_t now = cycleCount();
    uint32_t dt = now - *lastreport;
    while (rt) {
        __disable_irq();
        uint32_t cum = rt->cum; rt->cum = 0;// accumulated runtime
        uint32_t cnt = rt->cnt; rt->cnt = 0; // number of times stopped
        uint32_t max = rt->max; rt->max = 0; // max time
        __enable_irq();
        if (cnt == 0) {
            printf("%12s inactive\n", rt->name);
        } else {
            uint64_t period = dt / (C_US * cnt);
            uint64_t duty = cum / (C_US * cnt);

            printf("%12s #%ld p:%lld us avg:%lld us max %ld us\n", rt->name, cnt, period, duty, max/C_US);
        }
        rt = rt->next;
    }
    *lastreport = now;
}