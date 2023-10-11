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
#include "msgq.h"

#define printf tprintf

/* clang-format off */
enum {
    BMI_INT1A_PIN  = PA1,
	USART2_TX_PIN  = PA2,
    BMI_INT3G_PIN  = PA3,
    THERMISTOR_PIN = PA4,
    HEATER_EN_PIN  = PA5,
    INA_ALERT_PIN  = PA7,
	USART1_TX_PIN  = PA9,
	USARTR_TX_PIN  = PA10,
    TIMEPULSE_PIN  = PA15,

    BMI_CSB2G_PIN = PB0,
    BMI_CSB1A_PIN = PB1,
    SPI1_SCK_PIN  = PB3,
    SPI1_MISO_PIN = PB4,
    SPI1_MOSI_PIN = PB5,
    BME_CSB_PIN   = PB6,
    INA_CSB_PIN   = PB7,
    
};

static const struct gpio_config_t {
    enum GPIO_Pin  pins;
    enum GPIO_Conf mode;
} pin_cfgs[] = {
    {PAAll, GPIO_ANALOG},
    {PBAll, GPIO_ANALOG},
    {USART2_TX_PIN, GPIO_AF7_USART123|GPIO_HIGH},
	{SPI1_MOSI_PIN | SPI1_SCK_PIN |SPI1_MISO_PIN, GPIO_AF5_SPI12|GPIO_HIGH},
    {BMI_INT1A_PIN | BMI_INT3G_PIN | INA_ALERT_PIN, GPIO_IPU},
    {BMI_CSB1A_PIN | BMI_CSB2G_PIN | BME_CSB_PIN |INA_CSB_PIN, GPIO_OUTPUT|GPIO_FAST},
    {0, 0}, // sentinel
};

enum { IRQ_PRIORITY_GROUPING_2_2 = 5 }; // prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
#define PRIO(grp, sub) (((grp)<<6)|((sub)<<4))
struct {
    enum IRQn_Type irq;
    uint8_t        prio;
} irqprios[] = {
    {SysTick_IRQn,  PRIO(0,0)},
	{DMA2_CH6_IRQn, PRIO(1,0)},
	{DMA1_CH2_IRQn, PRIO(1,1)},
    {USART2_IRQn,   PRIO(2,0)},
    {USART1_IRQn,   PRIO(2,1)},
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
#if 0
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
#endif
/* clang-format on */

// USART2 is the console, for debug messages, it runs IRQ driven.
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

// SPI1 has the BMI088, BME280, INA299  ... connected to it.

struct SPIQ spiq1;

static void spi1_ss(uint16_t addr, int on) {
	switch ((enum BMXFunction)addr) {
	case NONE:  break;
	case ACCEL:	if (on) digitalLo(BMI_CSB1A_PIN); else digitalHi(BMI_CSB1A_PIN); break;
	case GYRO:	if (on) digitalLo(BMI_CSB2G_PIN); else digitalHi(BMI_CSB2G_PIN); break;
	case HUMID:	if (on) digitalLo(BME_CSB_PIN);   else digitalHi(BME_CSB_PIN); break;
	case CURRSENSE:	if (on) digitalLo(INA_CSB_PIN);   else digitalHi(INA_CSB_PIN); break;
	}
	printf("%d %s\n", addr, on ? "ON" : "OFF");
}

 void DMA1_CH2_Handler(void) {
	DMA1.IFCR = DMA1.ISR & 0x00000f0;
	spi_rx_dma_handler(&spiq1);
 }

// USART1 is the output datastream and command input

static struct MsgQueue outq;

static inline void usart1dmamove(uint8_t *buf, size_t len) {
    DMA2.CCR6 = 0;
    dma1_cselr_set_c6s(&DMA2, 2); // select usart1 for dma2 ch2
    DMA2.CPAR6 = (uintptr_t)&USART1.TDR;
    DMA2.CMAR6 = (uintptr_t)buf;
    DMA2.CNDTR6 = len;
    DMA2.CCR6 = DMA1_CCR1_MINC | DMA1_CCR1_DIR | DMA1_CCR1_TEIE | DMA1_CCR1_TCIE | DMA1_CCR1_EN;

}

// if we ever give up all messages being 20 bytes long, we need to introduce a 'padding' message
// to round up all packets to 960 or whatever given number of bytes, so we can stream out without
// having to buffer. 

static uint8_t usart1msgcnt = 48;
static uint16_t usart1chksum = 0;

// chksum of preceding packets, magic header and 960 bytes following (48 packets of 20 bytes)
// the upper limit is 1024 bytes but we want to leave some headroom
static uint8_t packetseparator[] = {0, 0, 'I', 'R', 'O', 'N', 0x03, 0xC0};

static int xmitmsg() {
    if (usart1msgcnt == 48) {
        packetseparator[0] = usart1chksum >> 8;
        packetseparator[1] = usart1chksum;
        usart1chksum = 0;
        usart1msgcnt = 0;
        usart1dmamove(&packetseparator[0], sizeof packetseparator);
    } else {
        struct Msg *msg = msgq_tail(&outq);
        if (msg == NULL) {
            return 0;
        }

        msg->buf[3] = msg->len;  // should be 20, assert?
        for (size_t i = 0; i < msg->len; ++i)
            usart1chksum += msg->buf[i];

        usart1dmamove(msg->buf, msg->len);
        ++usart1msgcnt;
    }

    USART1.ICR |= USART1_ICR_TCCF;  // clear Transmission Complete status
    USART1.CR1 |= USART1_CR1_UE | USART1_CR1_TE | USART1_CR1_TCIE;  // enable unit, TX and irq on TX Complete

    return 1;
}

uint32_t dma2ch6err_cnt = 0;
// dma error or complete
void DMA2_CH6_Handler(void) {
    if (DMA2.ISR & DMA1_ISR_TEIF6)
        ++dma2ch6err_cnt;

    DMA2.IFCR = DMA2.ISR & 0x00f00000;
}

// irq only on USART Transmit Complete
void USART1_Handler(void) {
    uint32_t isr = USART1.ISR;
    if ((isr & USART1_ISR_TC) == 0)
        return;

    USART1.ICR |= USART1_ICR_TCCF;  // clear Transmission Complete status

    // cnt is only zero after we sent a separator packet
    if (usart1msgcnt) {
        msgq_pop_tail(&outq);  // mark done
    }

    if (!xmitmsg()) {
        USART1.CR1 &= ~USART1_CR1_TCIE;
    }
}

void start_usart1(void) {
    if ((USART1.CR1 & USART1_CR1_TCIE) == 0)  // not already running
        xmitmsg();
}

volatile uint64_t dropped_usart1 = 0;  // queue full



// Timer 2: 1Hz status
void TIM2_Handler(void) {
    uint64_t now = cycleCount();

    if ((TIM2.SR & TIM1_SR_UIF) == 0)
            return;
    TIM2.SR &= ~TIM1_SR_UIF;

    now /= C_US; // microseconds
    uint64_t sec = now / 1000000;
    now %= 1000000;

//    printf("\nuptime %llu.%06llu  spiq %ld %ld %ld %04x %04x 0x%08lx\n", sec, now, spiq1.head, spiq1.curr, spiq1.tail, SPI1.CR1, SPI1.SR, DMA1.CNDTR2);
    printf("\nuptime %llu.%06llu  usart %ld-%ld (dropped %lld) cr:%04lx isr:%04lx dma len:0x%08lx e:%ld\n", sec, now, outq.head, outq.tail, dropped_usart1, USART1.CR1, USART1.ISR, DMA2.CNDTR6, dma2ch6err_cnt);
 }

extern uint32_t UNIQUE_DEVICE_ID[3]; // Section 47.1

void main(void) {
	uint8_t rf = (RCC.CSR >> 24) & 0xfc;
	RCC.CSR |= RCC_CSR_RMVF; // Set RMVF bit to clear the reset flags

	NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING_2_2);
    for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
        NVIC_SetPriority(irqprios[i].irq, irqprios[i].prio);
    }

	RCC.AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	RCC.AHB2ENR  |= RCC_AHB2ENR_GPIOAEN|RCC_AHB2ENR_GPIOBEN;
	RCC.APB1ENR1 |= RCC_APB1ENR1_USART2EN | RCC_APB1ENR1_TIM2EN;
	RCC.APB2ENR  |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN;

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

    usart_init(&USART1, 8*115200);
    USART1.CR3 |= USART1_CR3_DMAT;  // enable DMA
    NVIC_EnableIRQ(DMA2_CH6_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);

if(0)
    for(;;) {
        delay(500*1000);

        struct Msg *msg = msgq_head(&outq);
        if (!msg) {
            ++dropped_usart1;
             continue;
        }
        msg_reset(msg);
        msg_append64(msg, 0x6f6c616d75636861);
        msg_append32(msg, 0x68610a);
        msgq_push_head(&outq);
        start_usart1();
    }



	spiq_init(&spiq1, &SPI1, 4, SPI1_DMA1_CH23, spi1_ss); // 4: 80MHz/32 = 2.5Mhz, 3: 80MHz/16 = 5MHz.


	uint8_t val = 0;
	uint16_t r = bmx_readreg(&spiq1, HUMID, BME280_REG_ID, &val);
	printf("bme280 id reads (%x): %x\n", r, val);

#if 0

	int bmi_ok = (bmi_accel_poweron(&spiq1) == 0);
    if (!bmi_ok) {
        printf("BMI088 not found.\n");
    } else {
        printf("BMI088 Accel enabled.\n");
        bmi_ok = (bmi088_self_test(&spiq1) == 0);
    }
    if (bmi_ok) {
        uint16_t r = bmi_accel_poweron(&spiq1);
        if (r) {
            printf("BMI088 Accel failed to reset (%x).\n", r);
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

	int bme_ok = 1;
    if (!(bmi_ok && bme_ok)) {
        printf("BMI or BME not functional, watchdog will reboot....\n");
    }
#endif

    for(;;)
    	__WFI();

}