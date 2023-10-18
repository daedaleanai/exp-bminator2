#include "cortex_m4.h"
#include "stm32l4xx.h"

#include "binary.h"
#include "clock.h"
#include "gpio2.h"
#include "nvic.h"
#include "tprintf.h"
#include "usart.h"
#include "spi.h"
#include "bmxspi.h"
#include "bmi08x.h"
#include "msgq.h"
#include "output.h"

#define printf tprintf

#ifndef __REVISION__
#define __REVISION__ "<REV>"
#endif


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
    {USART2_TX_PIN, GPIO_AF7_USART123|GPIO_HIGH},
	{SPI1_MOSI_PIN | SPI1_SCK_PIN |SPI1_MISO_PIN, GPIO_AF5_SPI12|GPIO_HIGH},
    {BMI_INT1A_PIN | BMI_INT3G_PIN | INA_ALERT_PIN, GPIO_IPU},
    {BMI_CSB1A_PIN | BMI_CSB2G_PIN | BME_CSB_PIN |INA_CSB_PIN, GPIO_OUTPUT|GPIO_FAST},
    {0, 0}, // sentinel
};

 // prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
enum { IRQ_PRIORITY_GROUPING_2_2 = 5 };
#define PRIO(grp, sub) (((grp)<<6)|((sub)<<4))
struct {
    enum IRQn_Type irq;
    uint8_t        prio;
} irqprios[] = {
    {SysTick_IRQn,  PRIO(0,0)},
	{DMA2_CH6_IRQn, PRIO(1,0)},
	{DMA1_CH2_IRQn, PRIO(1,1)},
    {EXTI1_IRQn,    PRIO(1,2)},
    {EXTI3_IRQn,    PRIO(1,3)},
    {USART2_IRQn,   PRIO(2,0)},
    {USART1_IRQn,   PRIO(2,1)},
    {TIM2_IRQn,     PRIO(3,0)},
    {EXTI9_5_IRQn,  PRIO(3,1)},
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
static struct bmx_config_t const accel_cfg[] = {
    {BMI08x_ACC_RANGE, BMI088_ACC_RANGE_3G},  // SEE NOTE ABOVE
    {BMI08x_ACC_CONF, BMI08x_ACC_CONF_1600HZ_OSR4},
    {BMI08x_INT1_IO_CONF, 0x0C},  // INT1 enable output, Active low, open drain
    {BMI08x_INT1_INT2_MAP_DATA, 0x04},  // Map int to int1
    {0xFF, 0},  // sentinel
};

static struct bmx_config_t const gyro_cfg[] = {
    {BMI08x_GYRO_RANGE, BMI08x_GYRO_RANGE_250DEG_S},  // SEE NOTE ABOVE
    {BMI08x_GYRO_BANDWIDTH, BMI08x_GYRO_BANDWIDTH_2000_532HZ},
    {BMI08x_INT3_INT4_IO_CONF, 0x02},  // INT3 Active low, open drain
    {BMI08x_INT3_INT4_IO_MAP, 0x01},  // INT mapped to INT3
    {BMI08x_GYRO_INT_CTRL, 0x80},  // Enable INT generation
    {0xFF, 0},  // sentinel
};

static struct bmx_config_t const humid_cfg[] = {
    {BME280_REG_CONFIG, BME280_CONFIG_TSB10|BME280_CONFIG_FLT16},
    {BME280_REG_CTRLHUM, BME280_CTRLHUM_H16},
    {BME280_REG_CTRLMEAS, BME280_CTRLMEAS_P16|BME280_CTRLMEAS_T16|BME280_CTRLMEAS_NORMAL},
    {0xFF, 0},  // sentinel
};
/* clang-format on */

// ACCEL: cmd, dum,  [0x12..0x24): 6 registers for xyz plus 3 for time + 2dum + stat + temp
// GYRO: cmd, 6 registers, 2dum, stat
// TEMP: cmd, dum, 2 registers. big endian for some reason
// Baro: d
static uint8_t accel_buf[20];  // ACCEL BMI085_ACC_X_LSB
static uint8_t gyro_buf[10];  // GYRO  BMI085_RATE_X_LSB
//static uint8_t humid_buf[14];  // BME280_DATA_0

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

void hexdump(size_t len, const uint8_t* ptr) {
    static const char* hexchar = "01234567890abcdef";
    for (size_t i = 0; i<len; ++i) {
        _putchar(' ');
        _putchar(hexchar[ptr[i]>>4]);
        _putchar(hexchar[ptr[i]&0xf]);
    }
}


// SPI1 has the BMI088, BME280, INA299  ... connected to it.

struct SPIQ spiq;
static uint64_t dropped_spi1 = 0;  // how often we tried to submit a spi1 xmit but the queue was full

static void spi1_ss(uint16_t addr, int on) {
	switch ((enum BMXFunction)addr) {
	case NONE:  break;
	case ACCEL:	if (on) digitalLo(BMI_CSB1A_PIN); else digitalHi(BMI_CSB1A_PIN); break;
	case GYRO:	if (on) digitalLo(BMI_CSB2G_PIN); else digitalHi(BMI_CSB2G_PIN); break;
//	case HUMID:	if (on) digitalLo(BME_CSB_PIN);   else digitalHi(BME_CSB_PIN); break; // for some reason PB6 breaks things on the nucleo
	case HUMID:	if (on) digitalLo(INA_CSB_PIN);   else digitalHi(INA_CSB_PIN); break; // test
	case CURRSENSE:	if (on) digitalLo(INA_CSB_PIN);   else digitalHi(INA_CSB_PIN); break;
	}
}

void DMA1_CH2_Handler(void) {
	DMA1.IFCR = DMA1.ISR & 0x00000f0;
	spi_rx_dma_handler(&spiq);
}

static void exti_handler(uint64_t now, enum GPIO_Pin pin, uint16_t addr, uint8_t firstreg, uint8_t* buf, size_t len) {
    if ((EXTI.PR1 & pin) == 0)
        return;
    EXTI.PR1 = pin;
    struct SPIXmit *x = spiq_head(&spiq);
    if (!x) {
        ++dropped_spi1;
        return;
    }

   buf[0] = firstreg | 0x80;

    x->ts = now;
    x->tag = firstreg;
    x->addr = addr;
    x->len = len;
    x->buf = buf;
    spiq_enq_head(&spiq);
}
void EXTI1_Handler(void) { exti_handler(cycleCount(), Pin_1, ACCEL, BMI08x_ACC_X_LSB, accel_buf, sizeof accel_buf); }
void EXTI3_Handler(void) { exti_handler(cycleCount(), Pin_3, GYRO,  BMI08x_RATE_X_LSB, gyro_buf, sizeof gyro_buf); }


// INA alert interrupt
// void EXTI9_5_Handler(void) { }



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

//    printf("\nuptime %llu.%06llu  spiq %ld %ld %ld %04x %04x 0x%08lx\n", sec, now, spiq.head, spiq.curr, spiq.tail, SPI1.CR1, SPI1.SR, DMA2.CNDTR3);
    printf("\nuptime %llu.%06llu  usart1 %ld-%ld (dropped %lld) cr:%04lx isr:%04lx dma len:0x%08lx e:%ld\n", sec, now, outq.head, outq.tail, dropped_usart1, USART1.CR1, USART1.ISR, DMA2.CNDTR6, dma2ch6err_cnt);
//      printf("\nuptime %llu.%06llu\n", sec, now);
 }

extern uint32_t UNIQUE_DEVICE_ID[3]; // Section 47.1

static const char* pplsrcstr[] = { "NONE", "MSI", "HSI16", "HSE" };

void main(void) {
	uint8_t rf = (RCC.CSR >> 24) & 0xfc;
	RCC.CSR |= RCC_CSR_RMVF; // Set RMVF bit to clear the reset flags

	NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING_2_2);
    for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
        NVIC_SetPriority(irqprios[i].irq, irqprios[i].prio);
    }

    // Enable all the devices we are going to need
	RCC.AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	RCC.AHB2ENR  |= RCC_AHB2ENR_GPIOAEN|RCC_AHB2ENR_GPIOBEN;
	RCC.APB1ENR1 |= RCC_APB1ENR1_USART2EN | RCC_APB1ENR1_TIM2EN;
	RCC.APB2ENR  |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN;

	for (const struct gpio_config_t* p = pin_cfgs; p->pins; ++p) {
		gpioConfig(p->pins, p->mode);
	}

	gpioLock(PAAll);
	gpioLock(PBAll);

    // deselect all (active low) chip select signals
    digitalHi(BMI_CSB1A_PIN | BMI_CSB2G_PIN | BME_CSB_PIN |INA_CSB_PIN);

    // prepare USART2 for console and debug messages
	ringbuffer_clear(&usart2tx);
	usart_init(&USART2, 115200);
	NVIC_EnableIRQ(USART2_IRQn);

	printf("SWREV:%s\n", __REVISION__);
	printf("CPUID:%08lx\n",  SCB.CPUID);
	printf("IDCODE:%08lx\n",  DBGMCU.IDCODE);
	printf("DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
	printf("RESET:%02x%s%s%s%s%s%s\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "", rf & 0x20 ? " IWDG" : "",
	          rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
    printf("PPLSRC: %s%s\n", pplsrcstr[rcc_pllcfgr_get_pllsrc(&RCC)], 
        (RCC.CR & RCC_CR_HSEBYP) && (rcc_pllcfgr_get_pllsrc(&RCC)==3) ? " (CK_IN)" :""
    );
	usart_wait(&USART2);

    // set up timer2 for a 1Hz hearbeat
	TIM2.DIER |= TIM1_DIER_UIE;
    TIM2.PSC = (CLOCKSPEED_HZ/10000) - 1;
    TIM2.ARR = 10000 - 1; // 10KHz/10000 = 1Hz
    TIM2.CR1 |= TIM1_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);

    // Prepare USART1 for high speed DMA driven output of the measurement data
    usart_init(&USART1, 8*115200);
    USART1.CR3 |= USART1_CR3_DMAT;  // enable DMA
    NVIC_EnableIRQ(DMA2_CH6_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);

    // Set up SPI1 for talking to all the connected chips.
	spiq_init(&spiq, &SPI1, 3, SPI1_DMA1_CH23, spi1_ss); // 4: 80MHz/32 = 2.5Mhz, 3: 80MHz/16 = 5MHz.

    if (bmi_accel_poweron(&spiq) == 0) {
        printf("BMI088 Accel enabled.\n");
    } else {
        printf("BMI088 not found.\n");
    } 
    int bmi_ok = (bmi088_self_test(&spiq) == 0);
    usart_wait(&USART2);

    if (bmi_accel_poweron(&spiq)) {
        printf("BMI088 Accel failed to reset.\n");
        bmi_ok = 0;
    }

    usart_wait(&USART2);

    if (bmx_config(&spiq, ACCEL, accel_cfg) != 0) {
        printf("error configuring BMI Accel.\n");
        bmi_ok = 0;
    }

    usart_wait(&USART2);
    if (bmx_check_config(&spiq, ACCEL, accel_cfg) != 0) {
        printf("BMI Accel not properly configured\n");
        bmi_ok = 0;
    }
    usart_wait(&USART2);
    if (bmx_config(&spiq, GYRO, gyro_cfg) != 0) {
        printf("error configuring BMI Gyro\n");
        bmi_ok = 0;
    }

    usart_wait(&USART2);
    if (bmx_check_config(&spiq, GYRO, gyro_cfg) != 0) {
        printf("BMI Gyro not properly configured\n");
        bmi_ok = 0;
    }
    if (bmi_ok) {
        printf("BMI Enabled\n");
    }   

    usart_wait(&USART2);

	int bme_ok = (bme280_self_test(&spiq, &bmeParam) == 0);
    if (bme_ok) {
        printf("T:");
        for (size_t i = 1; i < 4; ++i)
            printf(" %ld", bmeParam.T[i]);
        printf("\n");

        printf("P:");
        for (size_t i = 1; i < 10; ++i)
            printf(" %ld", bmeParam.P[i]);
        printf("\n");

        printf("H:");
        for (size_t i = 1; i < 7; ++i)
            printf(" %ld", bmeParam.H[i]);
        printf("\n");
    }
    usart_wait(&USART2);

    if (bmx_config(&spiq, HUMID, humid_cfg) != 0) {
        printf("error configuring BME humidity sensor\n");
        bme_ok = 0;
    }

    usart_wait(&USART2);
    if (bmx_check_config(&spiq, HUMID, humid_cfg) != 0) {
        printf("BME humidity sensor not properly configured.\n");
        bme_ok = 0;
    }

    if (!(bmi_ok && bme_ok)) {
        printf("BMI or BME not functional, watchdog will reboot....\n");
    }


    EXTI.IMR1 |= (BMI_INT1A_PIN | BMI_INT3G_PIN | INA_ALERT_PIN) & Pin_All;
    EXTI.FTSR1 |= (BMI_INT1A_PIN | BMI_INT3G_PIN | INA_ALERT_PIN) & Pin_All;
    NVIC_EnableIRQ(EXTI1_IRQn);   // Accelerometer ready interrupt
    NVIC_EnableIRQ(EXTI3_IRQn);   // Gyroscope ready interrupt
    // NVIC_EnableIRQ(EXTI9_5_IRQn); // Current sense alert interrupt

    // headers used in output message vary by accel/gyro configuration
    // gyro_hdr = EVENTID_GYRO_2000DEG_S - gyro_cfg[0].val;  // minus is not a mistake
    // accel_hdr = EVENTID_ACCEL_2G + accel_cfg[0].val;

    // Initialize the independent watchdog
    // IWDG.KR = 0x5555;  // enable watchdog config
    // IWDG.PR = 0;       // prescaler /4 -> 10khz
    // IWDG.RLR = 3200;   // count to 3200 -> 320ms timeout
    // IWDG.KR = 0xcccc;  // start watchdog countdown

    for(;;) {
    	__WFI();

        struct SPIXmit *x = spiq_tail(&spiq);
        if (x != NULL) {
            struct Msg *msg = msgq_head(&outq);
            if (!msg) {
                ++dropped_usart1;
            } else {
                msg_reset(msg);
                if (output(msg, x)) {
                    msgq_push_head(&outq);
                    start_usart1();
                }
            }
            spiq_deq_tail(&spiq);

            //IWDG.KR = 0xAAAA;  // pet the watchdog
        } 

    }

}