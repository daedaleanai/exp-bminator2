#include "cortex_m4.h"
#include "stm32l4xx.h"

#include "clock.h"
#include "gpio2.h"
#include "nvic.h"
#include "tprintf.h"
#include "usart.h"

#define printf tprintf

/* clang-format off */
enum {
	USART2_TX_PIN = PA2,
	LED_PIN      = PB3,
};

static const struct gpio_config_t {
    enum GPIO_Pin  pins;
    enum GPIO_Conf mode;
} pin_cfgs[] = {
    {LED_PIN, GPIO_OUTPUT},
    {USART2_TX_PIN, GPIO_AF7_USART123|GPIO_HIGH},
    {0, 0}, // sentinel
};

enum { IRQ_PRIORITY_GROUPING_2_2 = 5 }; // prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
#define PRIO(grp, sub) (((grp)<<6)|((sub)<<4))
struct {
    enum IRQn_Type irq;
    uint8_t        prio;
} irqprios[] = {
    {SysTick_IRQn,  PRIO(0,0)},
    {USART2_IRQn,   PRIO(2,0)},
    {TIM2_IRQn,     PRIO(3,0)},
    {None_IRQn, 0xff},
};
#undef PRIO
/* clang-format on */

extern uint32_t UNIQUE_DEVICE_ID[3]; // Section 47.1

static struct Ringbuffer usart2tx;

void _putchar(char character) {
	if (!ringbuffer_full(&usart2tx)) {
		ringbuffer_put_head(&usart2tx, character);
	} else {
		ringbuffer_clear(&usart2tx);
		for (const char* p = "!OVFL!"; *p != 0; ++p) {
			ringbuffer_put_head(&usart2tx, *p);
		}
	}
	usart_start(&USART2);
	return;
}

void USART2_Handler(void) { usart_irq_handler(&USART2, &usart2tx); }

void TIM2_Handler(void) {
    uint64_t now = cycleCount();

    if ((TIM2.SR & TIM1_SR_UIF) == 0)
            return;
    TIM2.SR &= ~TIM1_SR_UIF;

    now /= C_US; // microseconds
    uint64_t sec = now / 1000000;
    now %= 1000000;

	digitalToggle(LED_PIN);

    printf("\nuptime %llu.%06llu\n", sec, now);
 }

void main(void) {
	uint8_t rf = (RCC.CSR >> 24) & 0xfc;
	RCC.CSR |= RCC_CSR_RMVF; // Set RMVF bit to clear the reset flags

	NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING_2_2);
    for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
            NVIC_SetPriority(irqprios[i].irq, irqprios[i].prio);
    }

	RCC.AHB2ENR |= RCC_AHB2ENR_GPIOAEN|RCC_AHB2ENR_GPIOBEN;
	RCC.APB1ENR1 |= RCC_APB1ENR1_USART2EN | RCC_APB1ENR1_TIM2EN;

	for (const struct gpio_config_t* p = pin_cfgs; p->pins; ++p) {
		gpioConfig(p->pins, p->mode);
	}

	gpioLock(PAAll);
	gpioLock(PBAll);

	ringbuffer_clear(&usart2tx);
	usart_init(&USART2, 115200);
	NVIC_EnableIRQ(USART2_IRQn);

	printf("SWREV:%s\n", __REVISION__);
	printf("CPUID:%08lx\n",  SCB.CPUID);
	printf("IDCODE:%08lx\n",  DBGMCU.IDCODE);
	printf("DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
	printf("RESET:%02x%s%s%s%s%s%s\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "", rf & 0x20 ? " IWDG" : "",
	          rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");

	usart_wait(&USART2);

	TIM2.DIER |= TIM1_DIER_UIE;
    TIM2.PSC = (CLOCKSPEED_HZ/10000) - 1;
    TIM2.ARR = 10000 - 1; // 10KHz/10000 = 1Hz
    TIM2.CR1 |= TIM1_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);

    for(;;)
    	__WFI();

}