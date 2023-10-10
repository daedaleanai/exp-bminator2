#include "cortex_m4.h"
#include "stm32l4xx.h"

#include "clock.h"
#include "gpio2.h"
#include "nvic.h"
#include "tprintf.h"
#include "usart.h"
#include "spi.h"
#include "bmxspi.h"
#include "bmi08x.h"

#define printf tprintf

/* clang-format off */
enum {
    BMI_INT1A_PIN = PA0,
    BMI_INT3G_PIN = PA1,
	USART2_TX_PIN = PA2,
	BME_CSB_PIN   = PA4,
    SPI1_SCK_PIN  = PA5,
    SPI1_MISO_PIN = PA6,
    SPI1_MOSI_PIN = PA7,
    BMI_CSB2G_PIN = PB0,
    BMI_CSB1A_PIN = PB1,

	LED_PIN      = PB3,
};

static const struct gpio_config_t {
    enum GPIO_Pin  pins;
    enum GPIO_Conf mode;
} pin_cfgs[] = {
    {LED_PIN, GPIO_OUTPUT},
    {USART2_TX_PIN, GPIO_AF7_USART123|GPIO_HIGH},
	{SPI1_MOSI_PIN | SPI1_SCK_PIN, GPIO_AF5_SPI12|GPIO_HIGH},
    {SPI1_MISO_PIN, GPIO_IPU},
    {BMI_INT1A_PIN | BMI_INT3G_PIN, GPIO_IPU},
    {BMI_CSB1A_PIN | BMI_CSB2G_PIN | BME_CSB_PIN, GPIO_OUTPUT|GPIO_FAST},
    {0, 0}, // sentinel
};

enum { IRQ_PRIORITY_GROUPING_2_2 = 5 }; // prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
#define PRIO(grp, sub) (((grp)<<6)|((sub)<<4))
struct {
    enum IRQn_Type irq;
    uint8_t        prio;
} irqprios[] = {
    {SysTick_IRQn,  PRIO(0,0)},
	{DMA1_CH2_IRQn, PRIO(1,0)},
    {USART2_IRQn,   PRIO(2,0)},
    {TIM2_IRQn,     PRIO(3,0)},
    {None_IRQn, 0xff},
};
#undef PRIO

/*
  Configuration of the BMI and BMP sensors

 NOTE: careful when modifying these  tables, the range
 must come first since it is used to choose the EventID

    uint32_t gyro_hdr = EVENTID_GYRO_2000DEG_S - gyro_cfg[0].val;  // minus is not a mistake
    uint32_t accel_hdr = EVENTID_ACCEL_2G + accel_cfg[0].val;

*/
static struct bmx_config_t gyro_cfg[] = {
        {BMI08x_GYRO_RANGE, BMI08x_GYRO_RANGE_250DEG_S},  // SEE NOTE ABOVE
        {BMI08x_GYRO_BANDWIDTH, BMI08x_GYRO_BANDWIDTH_2000_532HZ},
        {BMI08x_INT3_INT4_IO_CONF, 0x02},  // INT3 Active low, open drain
        {BMI08x_INT3_INT4_IO_MAP, 0x01},  // INT mapped to INT3
        {BMI08x_GYRO_INT_CTRL, 0x80},  // Enable INT generation
        {0xFF, 0},  // sentinel
};

static struct bmx_config_t accel_cfg[] = {
        {BMI08x_ACC_RANGE, BMI088_ACC_RANGE_3G},  // SEE NOTE ABOVE
        {BMI08x_ACC_CONF, BMI08x_ACC_CONF_1600HZ_OSR4},
        {BMI08x_INT1_IO_CONF, 0x0C},  // INT1 enable output, Active low, open drain
        {BMI08x_INT1_INT2_MAP_DATA, 0x04},  // Map int to int1
        {0xFF, 0},  // sentinel
};


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

struct SPIQ spiq1;

static void spi1_ss(uint16_t addr, int on) {
	switch ((enum BMXFunction)addr) {
		case NONE:  digitalHi(BMI_CSB1A_PIN|BMI_CSB2G_PIN|BME_CSB_PIN); break;
		case ACCEL:	if (on) digitalLo(BMI_CSB1A_PIN); else digitalHi(BMI_CSB1A_PIN); break;
		case GYRO:	if (on) digitalLo(BMI_CSB2G_PIN); else digitalHi(BMI_CSB2G_PIN); break;
		case HUMID:	if (on) digitalLo(BME_CSB_PIN);   else digitalHi(BME_CSB_PIN); break;
	}
}

//  void DMA1_CH2_Handler(void) {
// 	DMA1.IFCR = DMA1.ISR & 0x00f0;
// 	spi_rx_dma_handler(&spiq1);
//  }

void main(void) {
	uint8_t rf = (RCC.CSR >> 24) & 0xfc;
	RCC.CSR |= RCC_CSR_RMVF; // Set RMVF bit to clear the reset flags

	NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING_2_2);
    for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
            NVIC_SetPriority(irqprios[i].irq, irqprios[i].prio);
    }

//	RCC.AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC.AHB2ENR |= RCC_AHB2ENR_GPIOAEN|RCC_AHB2ENR_GPIOBEN;
	RCC.APB1ENR1 |= RCC_APB1ENR1_USART2EN | RCC_APB1ENR1_TIM2EN /*| RCC_APB1ENR1_SPI1EN*/;

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

if(0) {

	spiq_init(&spiq1, &SPI1, 4, SPI1_DMA1_CH23, spi1_ss); // 4: 80MHz/32 = 2.5Mhz, 3: 80MHz/16 = 5MHz.

	int bmi_ok = (bmi_accel_poweron(&spiq1) == 0);
    if (!bmi_ok) {
        printf("BMI085 not found.\n");
    } else {
        printf("BMI085 Accel enabled.\n");
        bmi_ok = (bmi088_self_test(&spiq1) == 0);
    }
    if (bmi_ok) {
        uint16_t r = bmi_accel_poweron(&spiq1);
        if (r) {
            printf("BMI085 Accel failed to reset (%x).\n", r);
            bmi_ok = 0;
        }
    }
    if (bmi_ok) {
        uint16_t r = bmx_config(&spiq1, ACCEL, accel_cfg);
        if (r != 0) {
            printf("error configuring BMI Accel: %x\n", r);
            bmi_ok = 0;
        }
        r = bmx_config(&spiq1, GYRO, gyro_cfg);
        if (r != 0) {
            printf("error configuring BMI Gyro: %x\n", r);
            bmi_ok = 0;
        }
        printf("BMI Enabled\n");
    }

	int bmp_ok = 1;
    if (!(bmi_ok && bmp_ok)) {
        printf("BMI or BMP not functional, watchdog will reboot....\n");
    }

}

	TIM2.DIER |= TIM1_DIER_UIE;
    TIM2.PSC = (CLOCKSPEED_HZ/10000) - 1;
    TIM2.ARR = 10000 - 1; // 10KHz/10000 = 1Hz
    TIM2.CR1 |= TIM1_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);




    for(;;)
    	__WFI();

}