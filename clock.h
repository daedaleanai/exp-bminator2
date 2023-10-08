#pragma once

enum { CLOCKSPEED_HZ = 80000000, C_US = CLOCKSPEED_HZ / 1000000 };

void initCycleCount(void);
uint64_t cycleCount(void);

// usec = 10...1000000 (.01 ms ..  1s)
void delay(uint32_t usec);
