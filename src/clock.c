#include "cortex_m4.h"
#include "stm32l4xx.h"
#include "clock.h"

static volatile uint64_t clockticks = STK_LOAD_RELOAD+1;;

void initCycleCount(void) {
//	clockticks = STK_LOAD_RELOAD+1;
	stk_load_set_reload(&STK, STK_LOAD_RELOAD); // maximum value
	stk_val_set_current(&STK, STK_LOAD_RELOAD);
	STK.CTRL |= STK_CTRL_CLKSOURCE | STK_CTRL_TICKINT | STK_CTRL_ENABLE;
}

// when this irq has a higher priority than all userspace, we may consider update to be atomic
// clockticks holds the counts at the _end_ of the next systick,
// so cyclecount can just subtract current VAL (which counts down)
void SysTick_Handler(void) { clockticks += STK_LOAD_RELOAD + 1; }

// VAL counts down from LOAD in 24 bits
uint64_t cycleCount(void) { return clockticks - stk_val_get_current(&STK); }

// delay up to 1'000'000 usec
void delay(uint32_t usec) {
	uint64_t now = cycleCount();
	uint64_t then = now + C_US * usec;
	while (cycleCount() < then)
		__NOP(); // wait
}
