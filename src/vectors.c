#include "cortex_m4.h"
#include "stm32l4xx.h"

extern void _estack(void);	// fake definition, will be filled in by linker script.

void default_IRQ_Handler(void) {
	for (;;)
		__NOP();
}

// Core fault handlers
void Reset_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void NonMaskableInt_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void MemoryManagement_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_7_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_8_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_9_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_10_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SVCall_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DebugMonitor_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void Reserved_13_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));

//  Device specific Interrupts
void WWDG_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void PVD_PVM_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RTC_TAMP_STAMP_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RTC_WKUP_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void FLASH_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RCC_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI0_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI4_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_CH1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_CH2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_CH3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_CH4_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_CH5_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_CH6_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA1_CH7_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void ADC1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void CAN1_TX_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void CAN1_RX0_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void CAN1_RX1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void CAN1_SCE_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI9_5_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM1_BRK_TIM15_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM1_UP_TIM16_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM1_TRG_COM_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM1_CC_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C1_EV_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C1_ER_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C2_EV_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C2_ER_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SPI1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SPI2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void USART1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void USART2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void USART3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void EXTI15_10_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void RTC_ALARM_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void SPI3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void UART4_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM6_DACUNDER_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void TIM7_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_CH1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_CH2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_CH3_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_CH4_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_CH5_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void COMP_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void LPTIM1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void LPTIM2_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_CH6_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void DMA2_CH7_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void LPUART1_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C3_EV_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C3_ER_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void FPU_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void CRS_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));
void I2C4_EV_Handler(void) __attribute__((weak, alias("default_IRQ_Handler")));

__attribute__((section(".isr_vector"))) void (*const vector_table[])(void) = {
		_estack, Reset_Handler, NonMaskableInt_Handler, Reserved_3_Handler, MemoryManagement_Handler, BusFault_Handler,
		UsageFault_Handler, Reserved_7_Handler, Reserved_8_Handler, Reserved_9_Handler, Reserved_10_Handler, SVCall_Handler,
		DebugMonitor_Handler, Reserved_13_Handler, PendSV_Handler, SysTick_Handler,
		[16 + WWDG_IRQn]		   = WWDG_Handler,			  // 0 Window Watchdog interrupt
		[16 + PVD_PVM_IRQn]		   = PVD_PVM_Handler,		  // 1 PVD through EXTI line detection
		[16 + RTC_TAMP_STAMP_IRQn] = RTC_TAMP_STAMP_Handler,  // 2 Tamper and TimeStamp interrupts
		[16 + RTC_WKUP_IRQn]	   = RTC_WKUP_Handler,		  // 3 RTC Tamper or TimeStamp /CSS on LSE through EXTI
															  // line 19 interrupts
		[16 + FLASH_IRQn]		   = FLASH_Handler,			  // 4 Flash global interrupt
		[16 + RCC_IRQn]			   = RCC_Handler,			  // 5 RCC global interrupt
		[16 + EXTI0_IRQn]		   = EXTI0_Handler,			  // 6 EXTI Line 0 interrupt
		[16 + EXTI1_IRQn]		   = EXTI1_Handler,			  // 7 EXTI Line 1 interrupt
		[16 + EXTI2_IRQn]		   = EXTI2_Handler,			  // 8 EXTI Line 2 interrupt
		[16 + EXTI3_IRQn]		   = EXTI3_Handler,			  // 9 EXTI Line 3 interrupt
		[16 + EXTI4_IRQn]		   = EXTI4_Handler,			  // 10 EXTI Line4 interrupt
		[16 + DMA1_CH1_IRQn]	   = DMA1_CH1_Handler,		  // 11 DMA1 Channel1 global interrupt
		[16 + DMA1_CH2_IRQn]	   = DMA1_CH2_Handler,		  // 12 DMA1 Channel2 global interrupt
		[16 + DMA1_CH3_IRQn]	   = DMA1_CH3_Handler,		  // 13 DMA1 Channel3 interrupt
		[16 + DMA1_CH4_IRQn]	   = DMA1_CH4_Handler,		  // 14 DMA1 Channel4 interrupt
		[16 + DMA1_CH5_IRQn]	   = DMA1_CH5_Handler,		  // 15 DMA1 Channel5 interrupt
		[16 + DMA1_CH6_IRQn]	   = DMA1_CH6_Handler,		  // 16 DMA1 Channel6 interrupt
		[16 + DMA1_CH7_IRQn]	   = DMA1_CH7_Handler,		  // 17 DMA1 Channel 7 interrupt
		[16 + ADC1_IRQn]		   = ADC1_Handler,			  // 18 ADC1 and ADC2 global interrupt
		[16 + CAN1_TX_IRQn]		   = CAN1_TX_Handler,		  // 19 CAN1 TX interrupts
		[16 + CAN1_RX0_IRQn]	   = CAN1_RX0_Handler,		  // 20 CAN1 RX0 interrupts
		[16 + CAN1_RX1_IRQn]	   = CAN1_RX1_Handler,		  // 21 CAN1 RX1 interrupts
		[16 + CAN1_SCE_IRQn]	   = CAN1_SCE_Handler,		  // 22 CAN1 SCE interrupt
		[16 + EXTI9_5_IRQn]		   = EXTI9_5_Handler,		  // 23 EXTI Line5 to Line9 interrupts
		[16 + TIM1_BRK_TIM15_IRQn] = TIM1_BRK_TIM15_Handler,  // 24 Timer 15 global interrupt
		[16 + TIM1_UP_TIM16_IRQn]  = TIM1_UP_TIM16_Handler,	  // 25 Timer 16 global interrupt
		[16 + TIM1_TRG_COM_IRQn]   = TIM1_TRG_COM_Handler,	  // 26 TIM1 trigger and commutation interrupt
		[16 + TIM1_CC_IRQn]		   = TIM1_CC_Handler,		  // 27 TIM1 Capture Compare interrupt
		[16 + TIM2_IRQn]		   = TIM2_Handler,			  // 28 TIM2 global interrupt
		[16 + TIM3_IRQn]		   = TIM3_Handler,			  // 29 TIM3 global interrupt
		[16 + I2C1_EV_IRQn]		   = I2C1_EV_Handler,		  // 31 I2C1 event interrupt
		[16 + I2C1_ER_IRQn]		   = I2C1_ER_Handler,		  // 32 I2C1 error interrupt
		[16 + I2C2_EV_IRQn]		   = I2C2_EV_Handler,		  // 33 I2C2 event interrupt
		[16 + I2C2_ER_IRQn]		   = I2C2_ER_Handler,		  // 34 I2C2 error interrupt
		[16 + SPI1_IRQn]		   = SPI1_Handler,			  // 35 SPI1 global interrupt
		[16 + SPI2_IRQn]		   = SPI2_Handler,			  // 36 SPI2 global interrupt
		[16 + USART1_IRQn]		   = USART1_Handler,		  // 37 USART1 global interrupt
		[16 + USART2_IRQn]		   = USART2_Handler,		  // 38 USART2 global interrupt
		[16 + USART3_IRQn]		   = USART3_Handler,		  // 39 USART2 global interrupt
		[16 + EXTI15_10_IRQn]	   = EXTI15_10_Handler,		  // 40 EXTI Lines 10 to 15 interrupts
		[16 + RTC_ALARM_IRQn]	   = RTC_ALARM_Handler,		  // 41 RTC alarms through EXTI line 18 interrupts
		[16 + SPI3_IRQn]		   = SPI3_Handler,			  // 51 SPI3 global Interrupt
		[16 + UART4_IRQn]		   = UART4_Handler,			  // 52 UART4 global Interrupt
		[16 + TIM6_DACUNDER_IRQn]  = TIM6_DACUNDER_Handler,	  // 54 TIM6 global and DAC1 and 2 underrun error
															  // interrupts
		[16 + TIM7_IRQn]	 = TIM7_Handler,				  // 55 TIM7 global interrupt
		[16 + DMA2_CH1_IRQn] = DMA2_CH1_Handler,			  // 56 DMA2 Channel 1 global Interrupt
		[16 + DMA2_CH2_IRQn] = DMA2_CH2_Handler,			  // 57 DMA2 Channel 2 global Interrupt
		[16 + DMA2_CH3_IRQn] = DMA2_CH3_Handler,			  // 58 DMA2 Channel 3 global Interrupt
		[16 + DMA2_CH4_IRQn] = DMA2_CH4_Handler,			  // 59 DMA2 Channel 4 global Interrupt
		[16 + DMA2_CH5_IRQn] = DMA2_CH5_Handler,			  // 60 DMA2 Channel 5 global Interrupt
		[16 + COMP_IRQn]	 = COMP_Handler,				  // 64 COMP1 and COMP2 interrupts
		[16 + LPTIM1_IRQn]	 = LPTIM1_Handler,				  // 65 LP TIM1 interrupt
		[16 + LPTIM2_IRQn]	 = LPTIM2_Handler,				  // 66 LP TIM2 interrupt
		[16 + DMA2_CH6_IRQn] = DMA2_CH6_Handler,			  // 68 DMA2 Channel 6 global Interrupt
		[16 + DMA2_CH7_IRQn] = DMA2_CH7_Handler,			  // 69 DMA2 Channel 7 global Interrupt
		[16 + LPUART1_IRQn]	 = LPUART1_Handler,				  // 70 LPUART1 global interrupt
		[16 + I2C3_EV_IRQn]	 = I2C3_EV_Handler,				  // 72 I2C3 event interrupt
		[16 + I2C3_ER_IRQn]	 = I2C3_ER_Handler,				  // 73 I2C3 error interrupt
		[16 + FPU_IRQn]		 = FPU_Handler,					  // 81 Floating point interrupt
		[16 + CRS_IRQn]		 = CRS_Handler,					  // 82 CRS interrupt
		[16 + I2C4_EV_IRQn]	 = I2C4_EV_Handler,				  // 83 I2C4 event interrupt, wakeup through EXTI
};
