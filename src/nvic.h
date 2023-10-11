#pragma once

#include "stm32l4xx.h"

// sets the priority grouping field using the required unlock sequence.
// 0,1,2,3 : group prio 7:4, subprio -
// 4 : group prio 7:5 subprio   4
// 5 : group prio 7:6 subprio 5:4
// 6 : group prio 7   subprio 6:4
// 7 : group prio -   subprio 7:4
static inline void NVIC_SetPriorityGrouping(uint32_t val) {
	uint32_t r = SCB.AIRCR;
	r &= ~(SCB_AIRCR_VECTKEYSTAT | SCB_AIRCR_PRIGROUP);
	r |= (0x5FAUL << 16) | ((val & 0x7) << 8);
	SCB.AIRCR = r;
}

// bits 7:4 of pri are used as per grouping setting
static inline void NVIC_SetPriority(enum IRQn_Type IRQn, uint8_t pri) {
	if (IRQn < 0) {
		((uint8_t*)(&SCB.SHPR1))[IRQn + 12] = pri;
	} else {
		((uint8_t*)(&NVIC.IPR0))[IRQn] = pri;
	}
}

// These only work for the non-system interrupts (irn >= 0)
static inline void NVIC_EnableIRQ(enum IRQn_Type IRQn) { (&NVIC.ISER0)[IRQn >> 5] = 1U << (IRQn & 0x1F); }
static inline void NVIC_DisableIRQ(enum IRQn_Type IRQn) { (&NVIC.ICER0)[IRQn >> 5] = 1U << (IRQn & 0x1F); }
