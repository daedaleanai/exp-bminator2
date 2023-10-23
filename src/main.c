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

/* clang-format off */
enum {
    BMI_INT1A_PIN  = PA1,
	USART2_TX_PIN  = PA2,
    BMI_INT3G_PIN  = PA3,
    THERMISTOR_PIN = PA4,
    HEATER_EN_PIN  = PA5,
    CURRENT_SENSE_PIN = PA6,
	USART1_TX_PIN  = PA9,
	USARTR_TX_PIN  = PA10,
    TIMEPULSE_PIN  = PA15,

    BMI_CSB2G_PIN = PB0,
    BMI_CSB1A_PIN = PB1,
    SPI1_SCK_PIN  = PB3,
    SPI1_MISO_PIN = PB4,
    SPI1_MOSI_PIN = PB5,
    BME_CSB_PIN   = PB7,  // for some reason PB6 doesnt work on the nucleo prototype
    
};

static const struct gpio_config_t {
    enum GPIO_Pin  pins;
    enum GPIO_Conf mode;
} pin_cfgs[] = {
    {USART1_TX_PIN, GPIO_AF7_USART123|GPIO_HIGH},
    {USART2_TX_PIN, GPIO_AF7_USART123|GPIO_HIGH},
	{SPI1_MOSI_PIN | SPI1_SCK_PIN |SPI1_MISO_PIN, GPIO_AF5_SPI12|GPIO_HIGH},
    {BMI_INT1A_PIN | BMI_INT3G_PIN, GPIO_IPU},
    {BMI_CSB1A_PIN | BMI_CSB2G_PIN | BME_CSB_PIN, GPIO_OUTPUT|GPIO_FAST},
    {TIMEPULSE_PIN, GPIO_AF2_TIM12 },
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
    {TIM6_DACUNDER_IRQn, PRIO(0,1)},
	{DMA2_CH6_IRQn, PRIO(1,0)},
	{DMA1_CH2_IRQn, PRIO(1,1)},
    {USART1_IRQn,   PRIO(1,2)},
    {EXTI1_IRQn,    PRIO(1,3)},
    {EXTI3_IRQn,    PRIO(1,3)},
    {USART2_IRQn,   PRIO(2,0)},
    {None_IRQn, 0xff},
};
#undef PRIO

/*
  Configuration of the BMI and BMP sensors.
  See their respective datasheets. 

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
static uint8_t accel_buf[20];  // ACCEL BMI085_ACC_X_LSB
static uint8_t gyro_buf[10];  // GYRO  BMI085_RATE_X_LSB
static uint8_t humid_buf[1+BME280_DATA_LEN];  // HUMID BME280_DATA_REG

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

// helper for debugging
void hexdump(size_t len, const uint8_t* ptr) {
    static const char* hexchar = "01234567890abcdef";
    for (size_t i = 0; i<len; ++i) {
        _putchar(' ');
        _putchar(hexchar[ptr[i]>>4]);
        _putchar(hexchar[ptr[i]&0xf]);
    }
}

// SPI1 has the BMI088, BME280 ... connected to it.

struct SPIQ spiq;
static uint64_t dropped_spi1 = 0;  // how often we tried to submit a spi1 xmit but the queue was full

static void spi1_ss(uint16_t addr, int on) {
	switch ((enum BMXFunction)addr) {
	case NONE:  break;
	case ACCEL:	if (on) digitalLo(BMI_CSB1A_PIN); else digitalHi(BMI_CSB1A_PIN); break;
	case GYRO:	if (on) digitalLo(BMI_CSB2G_PIN); else digitalHi(BMI_CSB2G_PIN); break;
	case HUMID:	if (on) digitalLo(BME_CSB_PIN);   else digitalHi(BME_CSB_PIN); break;
    }
}

void DMA1_CH2_Handler(void) {
	DMA1.IFCR = DMA1.ISR & 0x00000f0; // clear pending flag
	spi_rx_dma_handler(&spiq);        // see spi.c
}

static void exti_handler(uint64_t now, enum GPIO_Pin pin, uint16_t addr, uint8_t firstreg, uint8_t* buf, size_t len) {
    if ((EXTI.PR1 & pin) != pin) // also works for pin == 0
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

// Timer 2 channel 1 has the shutter time sync input

static volatile uint64_t shutter_open_ts = 0;
static volatile uint64_t shutter_close_ts = 0;
static volatile uint64_t shutter_open_cnt = 0;
static volatile uint64_t shutter_close_cnt = 0;

inline static uint64_t unlatch(volatile uint64_t *v) {
    __disable_irq();
    uint64_t r = *v;
    *v = 0;
    __enable_irq();
    return r;
}

void TIM2_Handler(void) {
    uint16_t cnt = TIM2.CNT;
    uint64_t now = cycleCount();
    uint16_t sr = TIM2.SR;

    if ((sr & TIM2_SR_CC1IF )) {
        uint16_t latency = (cnt - TIM2.CCR1);  // * (TIM2.PSC+1)
        shutter_open_ts = now - latency;
        ++shutter_open_cnt;
    }
    if ((sr & TIM2_SR_CC2IF)) {
        uint16_t latency = (cnt - TIM2.CCR2);  // * (TIM2.PSC+1)
        shutter_close_ts = now - latency;
        ++shutter_close_cnt;
    }

    TIM2.SR &= ~sr;
}

// USART1 is the output datastream and command input

static struct MsgQueue outq = { 0,0, {} };
static volatile uint64_t dropped_usart1 = 0;  // queue full, msqq_head returned NULL

enum { FrameSize = 1020 }; // we send frames of this size
static uint16_t framelen = FrameSize; // start by sending the separator
static uint16_t framechk = 0;
static int deq_after_xmit = 0;
static struct Msg framesep;

// set up DMA transaction on DMA2 channel 6 from buf[:len] to usart1 tx
// if buf is zero, we send len zero bytes instead.
static inline void usart1dmamove(uint8_t *buf, size_t len) {
    static uint8_t zero = 0;
    DMA2.CCR6 = 0;
    dma1_cselr_set_c6s(&DMA2, 2); // select usart1 for dma2 ch6
    DMA2.CPAR6 = (uintptr_t)&USART1.TDR;
    DMA2.CMAR6 = (uintptr_t) (buf ? buf : &zero);
    DMA2.CNDTR6 = len;
    if (buf) {
        DMA2.CCR6 = DMA1_CCR1_DIR | DMA1_CCR1_TEIE | DMA1_CCR1_TCIE | DMA1_CCR1_EN | DMA1_CCR1_MINC;
    } else {
        DMA2.CCR6 = DMA1_CCR1_DIR | DMA1_CCR1_TEIE | DMA1_CCR1_TCIE | DMA1_CCR1_EN;
    }
}

static int xmitmsg() {
    struct Msg *msg = msgq_tail(&outq);
    if (msg == NULL) {
        return 0;
    }

    if (framelen + msg->len == FrameSize) {
        // send the separator: chksum, "IRON", framesize
        framesep.len = 0;
        msg_append16(&framesep, framechk);
        msg_append32(&framesep, 0x49524f4e);  // 'IRON'
        msg_append16(&framesep, FrameSize);
        usart1dmamove(framesep.buf, framesep.len);
        framechk = 0;
        framelen = 0;
    } else if (framelen + msg->len > FrameSize) {
        // insert a padding message of all zeros (no length or header)
        usart1dmamove(NULL, FrameSize - framelen);
        // all zeros, no need to update the framecheck
        framelen = FrameSize;
    } else {
        framelen += msg->len;
        for (size_t i = 0; i < msg->len; ++i) {
            framechk += msg->buf[i];
        }
        usart1dmamove(msg->buf, msg->len);
        deq_after_xmit = 1;    
    }

    USART1.ICR |= USART1_ICR_TCCF;  // clear Transmission Complete status
    USART1.CR1 |= USART1_CR1_TCIE;  // irq on TX Complete

    return 1;
}

// irq on dma error or complete
uint32_t dma2ch6err_cnt = 0;
void DMA2_CH6_Handler(void) {
    if (DMA2.ISR & DMA1_ISR_TEIF6)
        ++dma2ch6err_cnt;
    DMA2.IFCR = DMA2.ISR & 0x00f00000;
}

// irq only on USART Transmit Complete
void USART1_Handler(void) {
    uint32_t isr = USART1.ISR;
    if ((isr & USART1_ISR_TC) != 0) {
        USART1.ICR |= USART1_ICR_TCCF;  // clear Transmission Complete status

        if (deq_after_xmit) {
            msgq_pop_tail(&outq);  // mark done
            deq_after_xmit = 0;
        }

        if (!xmitmsg()) { // start next if available
            USART1.CR1 &= ~USART1_CR1_TCIE;
        }
    }

}

// Timer 6: 1Hz status
void TIM6_DACUNDER_Handler(void) {
    uint64_t now = cycleCount();

    if ((TIM6.SR & TIM1_SR_UIF) == 0)
        return;
    TIM6.SR &= ~TIM1_SR_UIF;

    exti_handler(now, 0, HUMID, BME280_DATA_REG, humid_buf, sizeof humid_buf);

    now /= C_US; // microseconds
    uint64_t sec = now / 1000000;
    now %= 1000000;

  printf("\nuptime %llu.%06llu  spiq %ld %ld %ld (%lld) %04x %04x 0x%08lx\n", sec, now, spiq.head, spiq.curr, spiq.tail,  dropped_spi1, SPI1.CR1, SPI1.SR, DMA2.CNDTR3);
//  printf("\nuptime %llu.%06llu  usart1 %ld-%ld (dropped %lld) cr:%04lx isr:%04lx dma len:0x%08lx e:%ld\n", sec, now, outq.head, outq.tail, dropped_usart1, USART1.CR1, USART1.ISR, DMA2.CNDTR6, dma2ch6err_cnt);
//   printf("\nuptime %llu.%06llu\n", sec, now);
 }

extern uint32_t UNIQUE_DEVICE_ID[3]; // Section 47.1, defined in .ld file

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
	RCC.APB1ENR1 |= RCC_APB1ENR1_USART2EN | RCC_APB1ENR1_TIM6EN;
	RCC.APB2ENR  |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN;

	for (const struct gpio_config_t* p = pin_cfgs; p->pins; ++p) {
		gpioConfig(p->pins, p->mode);
	}

	gpioLock(PAAll);
	gpioLock(PBAll);

    // deselect all (active low) chip select signals
    digitalHi(BMI_CSB1A_PIN | BMI_CSB2G_PIN | BME_CSB_PIN);

    // prepare USART2 for console and debug messages
	ringbuffer_clear(&usart2tx);
	usart_init(&USART2, 115200);
	NVIC_EnableIRQ(USART2_IRQn);

#ifdef __REVISION__
	printf("SWREV:%s\n", __REVISION__);
#endif
	printf("CPUID:%08lx\n",  SCB.CPUID);
	printf("IDCODE:%08lx\n",  DBGMCU.IDCODE);
	printf("DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
	printf("RESET:%02x%s%s%s%s%s%s\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "", rf & 0x20 ? " IWDG" : "",
	          rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
    printf("PPLSRC: %s%s\n", pplsrcstr[rcc_pllcfgr_get_pllsrc(&RCC)], 
        (RCC.CR & RCC_CR_HSEBYP) && (rcc_pllcfgr_get_pllsrc(&RCC)==3) ? " (CK_IN)" :""
    );
	usart_wait(&USART2);

    // Prepare USART1 for high speed DMA driven output of the measurement data
    usart_init(&USART1, 921600);
    USART1.CR3 |= USART1_CR3_DMAT;  // enable DMA output
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

    EXTI.IMR1 |= (BMI_INT1A_PIN | BMI_INT3G_PIN) & Pin_All;
    EXTI.FTSR1 |= (BMI_INT1A_PIN | BMI_INT3G_PIN) & Pin_All;
    NVIC_EnableIRQ(EXTI1_IRQn);   // Accelerometer ready interrupt
    NVIC_EnableIRQ(EXTI3_IRQn);   // Gyroscope ready interrupt

    // TIM2 CH1 measures shutter open/close
    // See RM0394 section 27.3.5
    // CC1 channel is configured as input, IC1 is mapped on TI1,
    // CC2 channel is configured as input, IC2 is mapped on TI1
    TIM2.CR1 = 2 << 8;  // CKD = /4, 20MHz
    TIM2.DIER = TIM2_DIER_CC1IE | TIM2_DIER_CC2IE;
    TIM2.PSC = 0;  // 80MHz
    TIM2.ARR = 0xffff;
    TIM2.CCMR1_Input = (9 << 12) | (9 << 4) | (2 << 8) | 1;  //  fSAMPLING=fDTS/8, N=8
    TIM2.CCMR2_Input = 0;
    TIM2.CCER = TIM2_CCER_CC1E | TIM2_CCER_CC2E | TIM2_CCER_CC2P;  // 1,2 enabled, 2 inverted
    TIM2.CR1 |= TIM2_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);

    // set up TIM6 for a 1Hz hearbeat
    // note: it pushes spiq messages so do not start this before the BMI/BME are configured.
	TIM6.DIER |= TIM6_DIER_UIE;
    TIM6.PSC = (CLOCKSPEED_HZ/10000) - 1;
    TIM6.ARR = 10000 - 1; // 10KHz/10000 = 1Hz
    TIM6.CR1 |= TIM6_CR1_CEN;
    NVIC_EnableIRQ(TIM6_DACUNDER_IRQn);

    // headers used in output message vary by accel/gyro configuration
    gyro_hdr = EVENTID_GYRO_2000DEG_S - gyro_cfg[0].val;  // minus is not a mistake
    accel_hdr = EVENTID_ACCEL_2G + accel_cfg[0].val;

    // Initialize the independent watchdog
    IWDG.KR = 0x5555;  // enable watchdog config
    IWDG.PR = 0;       // prescaler /4 -> 10khz
    IWDG.RLR = 3200;   // count to 3200 -> 320ms timeout
    IWDG.KR = 0xcccc;  // start watchdog countdown

    for(;;) {
    	__WFI();

        struct Msg *msg = msgq_head(&outq);
        struct SPIXmit *x = spiq_tail(&spiq);
        if (x != NULL) {
            if (!msg) {
                ++dropped_usart1;
            } else {
                msg_reset(msg);
                if (output(msg, x)) {
                    msgq_push_head(&outq);
                    USART1.CR1 |= USART1_CR1_TCIE; // start USART1 if neccesary

                    IWDG.KR = 0xAAAA;  // pet the watchdog TODO check all subsystems
                }
            }
            spiq_deq_tail(&spiq);
        } 

        uint64_t ts = 0;
        if ((ts = unlatch(&shutter_open_ts)) != 0) {
            if (!msg) {
                ++dropped_usart1;
            } else {
                msg_reset(msg);
                msg_append16(msg, 0);  // len
                msg_append16(msg, EVENTID_SHUTTER_OPEN);  // header
                msg_append64(msg, ts);
                msg_append64(msg, shutter_open_cnt);         
                msg->buf[1] = msg->len;
            }
        } else if ((ts = unlatch(&shutter_close_ts)) != 0) {
            if (!msg) {
                ++dropped_usart1;
            } else {
                msg_reset(msg);
                msg_append16(msg, 0);  // len
                msg_append16(msg, EVENTID_SHUTTER_CLOSE);
                msg_append64(msg, ts);
                msg_append64(msg, shutter_close_cnt);
                msg->buf[1] = msg->len;
            }
        }

        if((msg != NULL) && (ts != 0)) {
            msgq_push_head(&outq);
            USART1.CR1 |= USART1_CR1_TCIE; // start USART1 if neccesary
        }

        // if we received a command, 


    }

}