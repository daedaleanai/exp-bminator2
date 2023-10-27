#include "cortex_m4.h"
#include "stm32l4xx.h"
#include "clock.h"

extern void main(void);                            // in main.c
extern void (*vector_table[])(void);               // in vector.c
extern char _sidata, _sdata, _edata, _sbss, _ebss; // provided by linker script

// how many clock cycles to wait before deciding not to use HSE (CK_IN)
enum { HSE_RDY_TIMEOUT = 40000 };  

// This is the first thing that runs after the CPU comes out of reset.
// It loads the data segment, clears the Blank Stuff segment,
// sets up the interrupt vector table to load from flash,
// initializes fault handling and FPU access,
// sets up the system clock to run at 80MHz, starts the system 
// timer and calls main().
void Reset_Handler(void) {

	// Copy data segment to RAM (see stm32XXXX.ld)
	char* src = &_sidata;
	char* dst = &_sdata;

	while (dst < &_edata)
		*dst++ = *src++;

	// clear BSS segment (see stm32XXXX.ld)
	for (dst = &_sbss; dst < &_ebss; dst++)
		*dst = 0;

	// Vector Table Relocation in Internal FLASH
	// PM0214 sec 4.4.4
	SCB.VTOR = (uintptr_t)&vector_table; // provided in vectors.c

	// PM0214 sec 4.4.7
	SCB.CCR |= SCB_CCR_DIV_0_TRP; // division by zero causes trap
	// enable usage/bus/mem fault separate handlers (in fault.c)
	// PM0214 sec 4.4.9
	SCB.SHCSR |= SCB_SHCSR_USGFAULTENA|SCB_SHCSR_BUSFAULTENA|SCB_SHCSR_MEMFAULTENA;

	// section 4.6.1 CP10/CP11 (FPU) Full Access
	fpu_cpacr_cpacr_set_cp(&FPU_CPACR, 0xf);  	 

	// Disable all interrupts and clear pending bits 
	RCC.CIER = 0;
	RCC.CICR = RCC.CIFR;

	// See RM0394 Section 6 for details on configuration of the clock tree.

	// feeding all peripherals to run at 80Mhz
	rcc_cfgr_set_hpre(&RCC, 0);  // AHB HCLK = SYSCLK  =  80MHz
	rcc_cfgr_set_ppre1(&RCC, 0); // APB1 PCLK = AHB HCLK 
	rcc_cfgr_set_ppre2(&RCC, 0); // APB2 PCLK = AHB HCLK 

	// set system clock to PLL 80 MHz, fed by MSI at 4MHz, or CK_IN(pa0) 8MHz
	// wait for HSE ready for a few ms, if not, fall back to MSI
	// tested closing SB17 on the STM32L432k nucleo board

	// enable HSE on CK_IN and wait for ready
	RCC.CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
	for (uint32_t t = 0; t < HSE_RDY_TIMEOUT; t++)
		if (RCC.CR & RCC_CR_HSERDY)
			break;

	if (RCC.CR & RCC_CR_HSERDY) {
		// if we got HSE ready, use it
		rcc_pllcfgr_set_pllsrc(&RCC, 3); // select 3:HSE (CK_IN via HSEBYPASS, assume 8MHz)
		rcc_pllcfgr_set_pllm(&RCC, 1);     // 0..15       : vco_in = HSE / (1+m)  4..16MHz        8/2 = 4MHz
	} else {
		// otherwise fall back to MSI
		RCC.CR &= ~(RCC_CR_HSEBYP | RCC_CR_HSEON);
		rcc_pllcfgr_set_pllsrc(&RCC, 1);   // select 1:MSI source (4MHz)
		rcc_pllcfgr_set_pllm(&RCC, 0);     // 0..15       : vco_in = MSI / (1+m)  4..16MHz        4/1 = 4MHz
	}

	rcc_pllcfgr_set_plln(&RCC, 40);     // 8...86     : vco_out = vco_in * n = 64...344MHz    4 * 40 = 160MHz
	rcc_pllcfgr_set_pllr(&RCC, 0); 		// 0,1,2,3 -> p=2,4,6,8  : sysclk = vco_out / p <= 170MHz  4 * 40 / 2 = 80MHz
	RCC.PLLCFGR |= RCC_PLLCFGR_PLLREN;  // emable R output (system clock)
	RCC.CR |= RCC_CR_PLLON;             // switch on the PLL

	// prepare the flash, RM0394 section 3.3.3
	FLASH.ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
	flash_acr_set_latency(&FLASH, 4);  // 4 wait states (5 cycles) cf 3.3.3 p79 table 9 
	while(flash_acr_get_latency(&FLASH) != 4)
		__NOP();

	// Wait till PLL is ready
	while ((RCC.CR & RCC_CR_PLLRDY) == 0)
		__NOP();

	// Select PLL as system clock source and wait until it takes effect
	rcc_cfgr_set_sw(&RCC, 3); 
	while (rcc_cfgr_get_sws(&RCC) != 3)
		__NOP();

	// Prepare the Cortex system timer
	stk_load_set_reload(&STK, STK_LOAD_RELOAD); // maximum value
	stk_val_set_current(&STK, STK_LOAD_RELOAD);
	STK.CTRL |= STK_CTRL_CLKSOURCE | STK_CTRL_TICKINT | STK_CTRL_ENABLE;

	main();

	for (;;)
		__NOP(); // hang
}

volatile uint64_t clockticks = STK_LOAD_RELOAD+1;

// when this irq has a higher priority than all userspace, we may consider update to be atomic
// clockticks holds the counts at the _end_ of the next systick,
// so cyclecount can just subtract current VAL (which counts down)
void SysTick_Handler(void) { clockticks += STK_LOAD_RELOAD + 1; }
