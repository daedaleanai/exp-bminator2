#pragma once

// This firmware uses the Cortex M4 builtin 'Systick' as a timer that counts in clock cycles.
// See PM0214 Section 4.5

enum { CLOCKSPEED_HZ = 80000000, C_US = CLOCKSPEED_HZ / 1000000 };

// Start the systick timer, typically first thing to do after the system clock is configured.
void initCycleCount(void);

// return the current time as measured in cpu cycles since initCycleCount was called.
uint64_t cycleCount(void);

// spinlock using the systick
// usec = 10...1000000 (.01 ms ..  1s)
void delay(uint32_t usec);
