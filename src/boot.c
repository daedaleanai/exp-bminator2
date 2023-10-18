#include "cortex_m4.h"
#include "stm32l4xx.h"
#include "clock.h"

extern void main(void);                            // in main.c
extern void (*vector_table[])(void);               // in vector.c
extern char _sidata, _sdata, _edata, _sbss, _ebss; // provided by linker script

enum { HSE_RDY_TIMEOUT = 5000 };

void Reset_Handler(void) {
	char* src = &_sidata;
	char* dst = &_sdata;

	while (dst < &_edata)
		*dst++ = *src++;

	for (dst = &_sbss; dst < &_ebss; dst++)
		*dst = 0;

	SCB.VTOR = (uintptr_t)&vector_table; // Vector Table Relocation in Internal FLASH.

	SCB.CCR |= SCB_CCR_DIV_0_TRP; // division by zero causes trap
	// enable usage/bus/mem fault separate handlers (in fault.c)
	SCB.SHCSR |= SCB_SHCSR_USGFAULTENA|SCB_SHCSR_BUSFAULTENA|SCB_SHCSR_MEMFAULTENA;

	fpu_cpacr_cpacr_set_cp(&FPU_CPACR, 0xf);  	// CP10/CP11 Full Access

	// Disable all interrupts and clear pending bits 
	RCC.CIER = 0;
	RCC.CICR = RCC.CIFR;

	// feeding all peripherals to run at 80Mhz
	rcc_cfgr_set_hpre(&RCC, 0);  // AHB HCLK = SYSCLK  =  80MHz
	rcc_cfgr_set_ppre1(&RCC, 0); // APB1 PCLK = AHB HCLK 
	rcc_cfgr_set_ppre2(&RCC, 0); // APB2 PCLK = AHB HCLK 

	// set system clock to PLL 80 MHz, fed by MSI at 4MHz, or CK_IN(pa0) 8MHz
	// wait for HSE ready for a few ms, if not, fall back to MSI
	// tested closing SB17 on the STM32L432k nucleo board

	// enable HSE on CK_IN and wait for ready
	RCC.CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
	for (uint32_t t = 0; t < 400000; t++)
		if (RCC.CR & RCC_CR_HSERDY)
			break;

	if (RCC.CR & RCC_CR_HSERDY) {
		rcc_pllcfgr_set_pllsrc(&RCC, 3); // select 3:HSE (CK_IN via HSEBYPASS)
		rcc_pllcfgr_set_pllm(&RCC, 1);     // 0..15       : vco_in = HSE / (1+m)  4..16MHz        8/2 = 4MHz
	} else {
		RCC.CR &= ~(RCC_CR_HSEBYP | RCC_CR_HSEON);
		rcc_pllcfgr_set_pllsrc(&RCC, 1);   // select 1:MSI source (4MHz)
		rcc_pllcfgr_set_pllm(&RCC, 0);     // 0..15       : vco_in = MSI / (1+m)  4..16MHz        4/1 = 4MHz
	}

	rcc_pllcfgr_set_plln(&RCC, 40);     // 8...86     : vco_out = vco_in * n = 64...344MHz    4 * 40 = 160MHz
	rcc_pllcfgr_set_pllr(&RCC, 0); 		// 0,1,2,3 -> p=2,4,6,8  : sysclk = vco_out / p <= 170MHz  4 * 40 / 2 = 80MHz
	RCC.PLLCFGR |= RCC_PLLCFGR_PLLREN;
	RCC.CR |= RCC_CR_PLLON;

	// prepare the flash	
	FLASH.ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
	flash_acr_set_latency(&FLASH, 4);  // 4 wait states (5 cycles) cf 3.3.3 p79 table 9 
	while(flash_acr_get_latency(&FLASH) != 4)
		__NOP();

	// Wait till PLL is ready
	while ((RCC.CR & RCC_CR_PLLRDY) == 0)
		__NOP();

	// Select PLL as system clock source
	rcc_cfgr_set_sw(&RCC, 3); 
	while (rcc_cfgr_get_sws(&RCC) != 3)
		__NOP();

	initCycleCount();  // so cycleCount() and delay() work

	main();

	for (;;)
		__NOP(); // hang
}
