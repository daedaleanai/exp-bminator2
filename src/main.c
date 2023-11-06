/*
  Main body of bminator 2 firmware.

  Theory of operation:
  - initialize pinout, irqs and peripherals
  - initialize and self-test BMI088 and BME380
  - set up irq handlers for accel, gyro, shutter, and periodic events
  - the accel and gyro irqs push SPI transactions on the spiq, which are handled by DMA
  - the other irqs push messages on the event queue evq
  - the mainloop empties both these queues and pushes output messages on the USART1 outq, which is also handled by DMA
  - incoming packets on USART1 are parsed by the USART1 irq and then handled in the main loop

*/
#include "cortex_m4.h"
#include "stm32l4xx.h"

#include <string.h>

#include "binary.h"
#include "bmi08x.h"
#include "bmxspi.h"
#include "clock.h"
#include "gpio2.h"
#include "msgq.h"
#include "nvic.h"
#include "output.h"
#include "runtimer.h"
#include "spi.h"
#include "tprintf.h"
#include "usart.h"

#define printf tprintf

enum {
	BMI_INT1A_PIN	  = PA1,
	USART2_TX_PIN	  = PA2,
	BMI_INT3G_PIN	  = PA3,
	THERMISTOR_PIN	  = PA4,  // ADC1_IN9
	HEATER_EN_PIN	  = PA5,
	CURRENT_SENSE_PIN = PA6,  // ADC1_IN11
	USART1_TX_PIN	  = PA9,
	USARTR_TX_PIN	  = PA10,
	TIMEPULSE_PIN	  = PA15,

	BMI_CSB2G_PIN = PB0,
	BMI_CSB1A_PIN = PB1,
	SPI1_SCK_PIN  = PB3,
	SPI1_MISO_PIN = PB4,
	SPI1_MOSI_PIN = PB5,
	BME_CSB_PIN	  = PB7,  // TODO for some reason PB6 doesnt work on the nucleo prototype
};

static struct gpio_config_t {
	enum GPIO_Pin  pins;
	enum GPIO_Conf mode;
} const pin_cfgs[] = {
		{ USART1_TX_PIN, GPIO_AF7_USART123 | GPIO_HIGH},
		{ USART2_TX_PIN, GPIO_AF7_USART123 | GPIO_HIGH},
		{ SPI1_MOSI_PIN | SPI1_SCK_PIN | SPI1_MISO_PIN, GPIO_AF5_SPI12 | GPIO_HIGH},
		{ BMI_INT1A_PIN | BMI_INT3G_PIN, GPIO_INPUT},
		{ BMI_CSB1A_PIN | BMI_CSB2G_PIN | BME_CSB_PIN, GPIO_OUTPUT | GPIO_FAST},
		{ TIMEPULSE_PIN, GPIO_INPUT },
        { THERMISTOR_PIN|CURRENT_SENSE_PIN, GPIO_ANALOG},
		{0, 0},	 // sentinel
};

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
		{BMI08x_INT1_IO_CONF, 0x08},		// INT1 enable output, Active low, push-pull
		{BMI08x_INT1_INT2_MAP_DATA, 0x04},	// Map int to int1
		{0xFF, 0},							// sentinel
};

static struct bmx_config_t const gyro_cfg[] = {
		{BMI08x_GYRO_RANGE, BMI08x_GYRO_RANGE_250DEG_S},  // SEE NOTE ABOVE
		{BMI08x_GYRO_BANDWIDTH, BMI08x_GYRO_BANDWIDTH_2000_532HZ},
		{BMI08x_INT3_INT4_IO_CONF, 0x00},  // INT3 Active low, push-pull
		{BMI08x_INT3_INT4_IO_MAP, 0x01},   // INT mapped to INT3
		{BMI08x_GYRO_INT_CTRL, 0x80},	   // Enable INT generation
		{0xFF, 0},						   // sentinel
};

static struct bmx_config_t const humid_cfg[] = {
		{BME280_REG_CONFIG, BME280_CONFIG_TSB10 | BME280_CONFIG_FLT16},
		{BME280_REG_CTRLHUM, BME280_CTRLHUM_H16},
		{BME280_REG_CTRLMEAS, BME280_CTRLMEAS_P16 | BME280_CTRLMEAS_T16 | BME280_CTRLMEAS_NORMAL},
		{0xFF, 0},	// sentinel
};


// prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
enum { IRQ_PRIORITY_GROUPING_2_2 = 5 };
#define PRIO(grp, sub) (((grp) << 6) | ((sub) << 4))

// irq handlers that push into the same queue must run at the same priority group
static struct {
	enum IRQn_Type irq;
	uint8_t		   prio;
} const irqprios[] = {
		{SysTick_IRQn, PRIO(0, 0)},
		{DMA1_CH2_IRQn, PRIO(1, 0)},	   // SPI RX done DMA
		{TIM6_DACUNDER_IRQn, PRIO(1, 0)},  // 8Hz periodic read BME shedules SPI xmit and push evq
		{EXTI1_IRQn, PRIO(1, 1)},		   // BMI088 Accel IRQ schedules SPI xmit
		{EXTI3_IRQn, PRIO(1, 1)},		   // BMI088 Gyro IRQ schedule SPI xmit
		{TIM2_IRQn, PRIO(1, 2)},		   // Shutter open/close timer, push evq
		{DMA2_CH7_IRQn, PRIO(2, 0)},	   // USART1 RX DMA
		{DMA2_CH6_IRQn, PRIO(2, 1)},	   // USART1 TX DMA done
		{USART1_IRQn, PRIO(2, 2)},		   // USART1 TX schedule next DMA
        {ADC1_IRQn, PRIO(3,0)},            // ADC conversions
		{USART2_IRQn, PRIO(3, 1)},		   // USART2 TX
        {TIM1_UP_TIM16_IRQn, PRIO(3,3)},   // TIM16 1Hz periodic debug report
		{None_IRQn, 0xff},				   // sentinel
};
#undef PRIO

static struct RunTimer spirxdma_rt    = { "SPIRXDMA", 0, 0, 0, 0, NULL};
static struct RunTimer periodic8hz_rt = { "8HZTICK", 0, 0, 0, 0, &spirxdma_rt};
static struct RunTimer accelirq_rt    = { "ACCELIRQ", 0,0,0,0, &periodic8hz_rt};
static struct RunTimer gyroirq_rt     = { "GYROIRQ", 0,0,0,0, &accelirq_rt};
static struct RunTimer shutterirq_rt  = { "SHUTTER", 0,0,0,0, &gyroirq_rt};
static struct RunTimer usart1rxdma_rt = { "USART1RXDMA", 0,0,0,0, &shutterirq_rt};
static struct RunTimer usart1txdma_rt = { "USART1TXDMA", 0,0,0,0, &usart1rxdma_rt};
static struct RunTimer usart1irq_rt   = { "USART1IRQ", 0,0,0,0, &usart1txdma_rt};
static struct RunTimer adcirq_rt      = { "ADCIRQ", 0,0,0,0, &usart1irq_rt};
static struct RunTimer report_rt      = { "REPORT", 0,0,0,0, &adcirq_rt};
static struct RunTimer mainloop_rt    = { "MAIN", 0,0,0,0, &report_rt};
static struct RunTimer idle_rt        = { "IDLE", 0,0,0,0, &mainloop_rt};
uint64_t lastreport = 0;

// USART2 is the console, for debug messages, it runs IRQ driven.
static struct Ringbuffer usart2tx;

void _putchar(char character) {
	if (!ringbuffer_full(&usart2tx)) {
		ringbuffer_put_head(&usart2tx, character);
	} else {
		// overflow, tell the user we are losing messages.
		ringbuffer_clear(&usart2tx);
		for (const char *p = "!OVFL!"; *p != 0; ++p) {
			ringbuffer_put_head(&usart2tx, *p);
		}
	}
	usart_start(&USART2);
	return;
}

void USART2_Handler(void) { usart_irq_handler(&USART2, &usart2tx); }

// helper for debugging
void hexdump(size_t len, const uint8_t *ptr) {
	static const char *hexchar = "01234567890abcdef";
	for (size_t i = 0; i < len; ++i) {
		_putchar(' ');
		_putchar(hexchar[ptr[i] >> 4]);
		_putchar(hexchar[ptr[i] & 0xf]);
	}
}

// SPI1 has the BMI088, BME280 ... connected to it.
// The gyro and accel irq and periodically TIM6 trigger a SPI transaction on the accel, gyro or humid
// sensor. The result of that transaction becomes available in the main loop.
static struct SPIQ spiq;
static volatile uint64_t dropped_spi1 = 0;  // how often we tried to submit a spi1 xmit but the queue was full

static void spi1_ss(uint16_t addr, int on) {
	switch ((enum BMXFunction)addr) {
	case NONE:
		digitalHi(BMI_CSB1A_PIN | BMI_CSB2G_PIN | BME_CSB_PIN);
		break;
	case ACCEL:
		if (on)
			digitalLo(BMI_CSB1A_PIN);
		else
			digitalHi(BMI_CSB1A_PIN);
		break;
	case GYRO:
		if (on)
			digitalLo(BMI_CSB2G_PIN);
		else
			digitalHi(BMI_CSB2G_PIN);
		break;
	case HUMID:
		if (on)
			digitalLo(BME_CSB_PIN);
		else
			digitalHi(BME_CSB_PIN);
		break;
	}
}

void DMA1_CH2_Handler(void) {
    rt_start(&spirxdma_rt, cycleCount());
	DMA1.IFCR = DMA1.ISR & 0x00000f0;  // clear pending flag
	spi_rx_dma_handler(&spiq);		   // see spi.c
    rt_stop(&spirxdma_rt, cycleCount());
}

static void start_spix(uint64_t now, uint16_t addr, uint8_t firstreg, uint8_t *buf, size_t len) {
	struct SPIXmit *x = spiq_head(&spiq);
	if (!x) {
		++dropped_spi1;
		return;
	}

	buf[0] = firstreg | 0x80;  // set the read flag on the register address.
    for (size_t i = 1; i < len; i++) {
        buf[i] = 0;
    }
    
	x->ts	= now;
	x->tag	= firstreg;
	x->addr = addr;
	x->len	= len;
	x->buf	= buf;
	spiq_enq_head(&spiq);
}

// ACCEL: cmd, dum,  [0x12..0x24): 6 registers for xyz plus 3 for time + 2dum + stat + temp
// GYRO: cmd, 6 registers, 2dum, stat
// TEMP: cmd, dum, 2 registers. big endian for some reason
static uint8_t accel_buf[20];					// ACCEL BMI085_ACC_X_LSB
static uint8_t gyro_buf[10];					// GYRO  BMI085_RATE_X_LSB
static uint8_t humid_buf[1 + BME280_DATA_LEN];	// HUMID BME280_DATA_REG

void EXTI1_Handler(void) {
	uint64_t now = cycleCount();
	if ((EXTI.PR1 & Pin_1) == 0)
		return;
	EXTI.PR1 = Pin_1;
	start_spix(now, ACCEL, BMI08x_ACC_X_LSB, accel_buf, sizeof accel_buf);
    rt_start(&accelirq_rt, now);
    rt_stop(&accelirq_rt, cycleCount());
}

void EXTI3_Handler(void) {
	uint64_t now = cycleCount();
	if ((EXTI.PR1 & Pin_3) == 0)
		return;
	EXTI.PR1 = Pin_3;
	start_spix(now, GYRO, BMI08x_RATE_X_LSB, gyro_buf, sizeof gyro_buf);
    rt_start(&gyroirq_rt, now);
    rt_stop(&gyroirq_rt, cycleCount());
}

// Other events go to another message queue, which the mainloop copies into the output stream.
static struct MsgQueue	 evq		 = {0, 0, {}};
static volatile uint64_t dropped_evq = 0;  // queue full, msqq_head returned NULL

extern uint32_t UNIQUE_DEVICE_ID[3];  // Section 47.1, defined in .ld file

static volatile int report = 0; // time to dump status in main loop
static volatile uint32_t tim6_tick = 0;
// Timer 6: 8Hz BME read, periodic messages, status report
void TIM6_DACUNDER_Handler(void) {
	uint64_t now = cycleCount();

	if ((TIM6.SR & TIM1_SR_UIF) == 0)
		return;
	TIM6.SR &= ~TIM1_SR_UIF;
    rt_start(&periodic8hz_rt, now);

	struct Msg *msg = msgq_head(&evq);

	switch (tim6_tick++ & 7) {
	case 0:
        report = 1; 
		break;

	case 1:
		start_spix(now, HUMID, BME280_DATA_REG, humid_buf, sizeof humid_buf);
		break;

	case 2:
		if (!msg) {
			++dropped_evq;
			break;
		}
		output_periodic(msg, EVENTID_ID0, now, __REVISION__, UNIQUE_DEVICE_ID[2]);
		msgq_push_head(&evq);
		break;

	case 3:
		if (!msg) {
			++dropped_evq;
			break;
		}
		output_periodic(msg, EVENTID_ID1, now, UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
		msgq_push_head(&evq);
		break;

	case 4:
	case 5:
	case 6:
	case 7:
		break;
	}
    rt_start(&periodic8hz_rt, now);
    rt_stop(&periodic8hz_rt, cycleCount());
}

static volatile uint64_t shutter_count = 0;	 //
// Timer 2 channel 1 has the shutter time sync input
void TIM2_Handler(void) {
	uint16_t cnt = TIM2.CNT;
	uint64_t now = cycleCount();
	uint16_t sr	 = TIM2.SR;
	TIM2.SR &= ~sr;

    rt_start(&shutterirq_rt, now);

	// these could have happened in any order
	uint64_t open_ts  = 0;
	uint64_t close_ts = 0;

	if ((sr & TIM2_SR_CC1IF)) {
		uint16_t latency = (cnt - TIM2.CCR1);  // * (TIM2.PSC+1)
		open_ts			 = now - latency;
	}

	if ((sr & TIM2_SR_CC2IF)) {
		uint16_t latency = (cnt - TIM2.CCR2);  // * (TIM2.PSC+1)
		close_ts		 = now - latency;
	}

    printf("open %lld close %lld\n", open_ts, close_ts);

	struct Msg *msg = msgq_head(&evq);

	if ((open_ts && close_ts) && (open_ts <= close_ts)) {
		shutter_count = (shutter_count + 1) | 1;  // next odd number.
		if (!msg) {
			++dropped_evq;
		} else {
			output_shutter(msg, EVENTID_SHUTTER_OPEN, open_ts, shutter_count);
			msgq_push_head(&evq);
			msg = msgq_head(&evq);
		}
		open_ts = 0;  // mark as not happened, or already sent
	}

	if (close_ts) {
		shutter_count = (shutter_count | 1) + 1;  // next even number.
		if (!msg) {
			++dropped_evq;
		} else {
			output_shutter(msg, EVENTID_SHUTTER_CLOSE, close_ts, shutter_count);
			msgq_push_head(&evq);
			msg = msgq_head(&evq);
		}
	}
	// if we didnt already send it before
	if (open_ts) {
		shutter_count = (shutter_count + 1) | 1;  // next odd number.
		if (!msg) {
			++dropped_evq;
		} else {
			output_shutter(msg, EVENTID_SHUTTER_OPEN, open_ts, shutter_count);
			msgq_push_head(&evq);
		}
	}

    rt_stop(&shutterirq_rt, cycleCount());
}

static volatile int adc_chan = 0;
static volatile uint16_t adc_val[5] = {0,0,0,0,0};
static volatile uint64_t adc_ts[5] = {0,0,0,0,0};

void ADC1_Handler(void) {
    uint64_t now = cycleCount();
    uint16_t isr = ADC.ISR;
    if (isr & ADC_ISR_EOC) {
        adc_val[adc_chan] = ADC.DR;
        adc_ts[adc_chan] = now;
        ++adc_chan;
    }
    if (isr & ADC_ISR_EOS) {
        adc_chan = 0;
    }
    rt_start(&adcirq_rt, now);
    rt_stop(&adcirq_rt, cycleCount());
}


// USART1 is the output datastream and command input

static struct MsgQueue	 outq				= {0, 0, {}};
static volatile uint64_t dropped_usart1		= 0;  // queue full, msqq_head returned NULL
static volatile uint32_t usart1txdmaerr_cnt = 0;
static volatile uint32_t usart1rxdmaerr_cnt = 0;

// USART1 TX: irq on dma error or complete
void DMA2_CH6_Handler(void) {
    rt_start(&usart1txdma_rt, cycleCount());
	uint32_t isr = DMA2.ISR;
	DMA2.IFCR	 = isr & 0x00f00000;
	if (isr & DMA1_ISR_TEIF6)
		++usart1txdmaerr_cnt;

	if (isr & DMA1_ISR_TCIF6) {
		if (msgq_tail(&outq))
			msgq_pop_tail(&outq);
	}
    rt_stop(&usart1txdma_rt, cycleCount());
}

void USART1_Handler(void) {
	// Transmission Complete
	if ((USART1.ISR & USART1_ISR_TC) == 0)
		return;

    rt_start(&usart1irq_rt, cycleCount());

	struct Msg *msg = msgq_tail(&outq);
	if (msg == NULL) {
		// queue empty, clear IRQ Enable
		USART1.CR1 &= ~USART1_CR1_TCIE;

	} else {

        // queue not empty, start a new transfer
        USART1.ICR |= USART1_ICR_TCCF;	// clear irq flag

        DMA2.CCR6 = 0;
        dma1_cselr_set_c6s(&DMA2, 2);  // select usart1 for dma2 ch6
        DMA2.CPAR6	= (uintptr_t)&USART1.TDR;
        DMA2.CMAR6	= (uintptr_t)msg->buf;
        DMA2.CNDTR6 = msg->len;
        DMA2.CCR6	= DMA1_CCR6_DIR | DMA1_CCR6_TEIE | DMA1_CCR6_TCIE | DMA1_CCR6_EN | DMA1_CCR6_MINC;
    }
      rt_stop(&usart1irq_rt, cycleCount());
}

static uint8_t		   cmdbuf[1024];
static volatile size_t cmdbuf_head							 = 0;
static volatile enum { RESYNC, MORE, COMPLETE } cmdbuf_state = RESYNC;
static volatile size_t cmdpacket_size						 = 0;

// set up usart1 rx dma for reception of len characters to be appended to cmdbuf
// set CC7 to run at higher priority level compared to ch6 (CCR PL bits)
void usart1_start_rx(size_t len) {
	DMA2.CCR7 = 0;
	dma1_cselr_set_c7s(&DMA2, 2);  // select usart1 for dma2 ch7
	DMA2.CPAR7 = (uintptr_t)&USART1.RDR;
	DMA2.CMAR7 = (uintptr_t)cmdbuf + cmdbuf_head;
	cmdbuf_head += len;
	DMA2.CNDTR7 = len;
	DMA2.CCR7	= DMA1_CCR7_PL | DMA1_CCR7_TEIE | DMA1_CCR7_TCIE | DMA1_CCR7_EN | DMA1_CCR7_MINC;
}

static inline size_t findbyte(uint8_t *buf, size_t buflen, uint8_t b) {
	size_t i = 0;
	for (; i < buflen; ++i)
		if (buf[i] == b)
			break;
	return i;
}

static inline int haspfx(uint8_t *buf, size_t buflen, uint8_t *pfx, size_t pfxlen) {
	if (buflen < pfxlen)
		return 0;
	while (pfxlen--)
		if (*buf++ != *pfx++)
			return 0;
	return 1;
}

static inline void handlerx() {
    switch (cmdbuf_state) {
    case RESYNC:
        while (!haspfx(cmdbuf, cmdbuf_head, "IRON", 4)) {
            size_t start = 1 + findbyte(cmdbuf + 1, cmdbuf_head - 1, 'I');
            memmove(cmdbuf, cmdbuf + start, cmdbuf_head - start);
            cmdbuf_head -= start;
            if (cmdbuf_head < 8) {
                usart1_start_rx(8 - cmdbuf_head);
                return;
            }

            // we have at least 8 bytes starting with 'IRON', next two bytes are packet size
            cmdpacket_size = decode_be_uint16(cmdbuf + 4);
            memmove(cmdbuf, cmdbuf + 6, cmdbuf_head - 6);
            cmdbuf_head -= 6;
            if (cmdpacket_size <= 1022)
                break;
        }

        cmdbuf_state = MORE;
        // fallthrough
    case MORE:
        if (cmdbuf_head < cmdpacket_size + 2) {
            usart1_start_rx(2 + cmdpacket_size - cmdbuf_head);
            return;
        }
        // now we have cmdpacket_size + 2 bytes in the buffer
        cmdbuf_state = COMPLETE;

    case COMPLETE:
    }
    return;
}

// USART1 RX: irq on dma error or complete
void DMA2_CH7_Handler(void) {
    rt_start(&usart1rxdma_rt, cycleCount());
	uint32_t isr = DMA2.ISR;
	DMA2.IFCR	 = isr & 0x0f000000;
	if (isr & DMA1_ISR_TEIF7)
		++usart1rxdmaerr_cnt;

	if (isr & DMA1_ISR_TCIF7) {
        handlerx();
	}
    rt_stop(&usart1rxdma_rt, cycleCount());
}

void TIM1_UP_TIM16_Handler(void) {
    uint64_t now = cycleCount();
    uint16_t sr	 = TIM16.SR;
	TIM16.SR &= ~sr;

    uint64_t us = now / C_US;  // microseconds

    printf("uptime %llu.%06llu\n", us / 1000000, us % 1000000);
    printf("enqueued spiq: %8lu evq:%8lu outq: %8lu\n", spiq.head, evq.head, outq.head);
    printf("dropped  spiq: %8llu evq:%8llu outq: %8llu\n", dropped_spi1, dropped_evq, dropped_usart1);
    printf("usart1 error tx: %ld rx:%ld\n", usart1txdmaerr_cnt, usart1rxdmaerr_cnt);
    rt_report(&idle_rt, &lastreport);

    // account for reporting time outside of rt_report call
    rt_start(&report_rt, now);
    rt_stop(&report_rt, cycleCount());
}

static const char *pplsrcstr[] = {"NONE", "MSI", "HSI16", "HSE"};

void main(void) {
	uint8_t rf = (RCC.CSR >> 24) & 0xfc;
	RCC.CSR |= RCC_CSR_RMVF;  // Set RMVF bit to clear the reset flags

	NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING_2_2);
	for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
		NVIC_SetPriority(irqprios[i].irq, irqprios[i].prio);
	}

	// Enable all the devices we are going to need
	RCC.AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	RCC.AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_ADCEN;
	RCC.APB1ENR1 |= RCC_APB1ENR1_USART2EN | RCC_APB1ENR1_TIM6EN | RCC_APB1ENR1_TIM2EN ;
	RCC.APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM16EN;

	for (const struct gpio_config_t *p = pin_cfgs; p->pins; ++p) {
		gpioConfig(p->pins, p->mode);
	}

	gpioLock(PAAll);
	gpioLock(PBAll);

	// deselect all (active low) chip select signals
    digitalHi(BMI_CSB1A_PIN | BMI_CSB2G_PIN | BME_CSB_PIN);
    spi1_ss(HUMID, 1); delay(5); spi1_ss(HUMID, 0); // force BME into SPI mode
    spi1_ss(ACCEL, 1); delay(5); spi1_ss(ACCEL, 0); // force BMI accel part into SPI mode

	// prepare USART2 for console and debug messages
	ringbuffer_clear(&usart2tx);
	usart_init(&USART2, 115200);
	NVIC_EnableIRQ(USART2_IRQn);

	printf("SWREV:%x\n", __REVISION__);
	printf("CPUID:%08lx\n", SCB.CPUID);
	printf("IDCODE:%08lx\n", DBGMCU.IDCODE);
	printf("DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
	printf("RESET:%02x%s%s%s%s%s%s\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "", rf & 0x20 ? " IWDG" : "",
		   rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
	printf("PPLSRC: %s%s\n", pplsrcstr[rcc_pllcfgr_get_pllsrc(&RCC)],
		   (RCC.CR & RCC_CR_HSEBYP) && (rcc_pllcfgr_get_pllsrc(&RCC) == 3) ? " (CK_IN)" : "");
	usart_wait(&USART2);

	// Set up SPI1 for talking to all the connected chips.
	spiq_init(&spiq, &SPI1, 4, SPI1_DMA1_CH23, spi1_ss);  // 4: 80MHz/32 = 2.5Mhz, 3: 80MHz/16 = 5MHz.

    // test and config BME280

    if (bme280_self_test(&spiq, &bmeParam) == 0) {
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
	}
	usart_wait(&USART2);


    // test and config BMI088

	if (bmi_accel_poweron(&spiq) == 0) {
		printf("BMI088 Accel enabled.\n");
	} else {
		printf("BMI088 not found.\n");
	}

	bmi088_self_test(&spiq);
	usart_wait(&USART2);

	if (bmi_accel_poweron(&spiq)) {
		printf("BMI088 Accel failed to reset.\n");
	}

	if (bmx_config(&spiq, ACCEL, accel_cfg) != 0) {
		printf("BMI088 error configuring Accel.\n");
	}

	if (bmx_config(&spiq, GYRO, gyro_cfg) != 0) {
		printf("BMI088 error configuring Gyro\n");
	}
    usart_wait(&USART2);


    int accel_ok = (bmx_check_config(&spiq, ACCEL, accel_cfg) == 0);
	if (!accel_ok) {
		printf("BMI Accel not properly configured\n");
	}
	usart_wait(&USART2);
    int gyro_ok  = (bmx_check_config(&spiq, GYRO, gyro_cfg) == 0);
	if (!gyro_ok) {
		printf("BMI Gyro not properly configured\n");
    }

	usart_wait(&USART2);
    int humid_ok = (bmx_check_config(&spiq, HUMID, humid_cfg) == 0);
	if (!humid_ok) {
		printf("BME humidity sensor not properly configured.\n");
	}
	usart_wait(&USART2);

	// Prepare USART1 for high speed DMA driven output of the measurement data
	usart_init(&USART1, 921600);
	USART1.CR3 |= USART1_CR3_DMAT | USART1_CR3_DMAR;  // enable DMA output and input
	usart1_start_rx(8);  // ask rx for the first 8 bytes
	NVIC_EnableIRQ(DMA2_CH6_IRQn);
	NVIC_EnableIRQ(DMA2_CH7_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);

	// TIM2 CH1 measures shutter open/close
	// See RM0394 section 27.3.5
	// CC1 channel is configured as input, IC1 is mapped on TI1,
	// CC2 channel is configured as input, IC2 is mapped on TI1
	TIM2.CR1		 = 2 << 8;	// CKD = /4, 20MHz
	TIM2.DIER		 = TIM2_DIER_CC1IE | TIM2_DIER_CC2IE;
	TIM2.PSC		 = 0;  // 80MHz
	TIM2.ARR		 = 0xffff;
	TIM2.CCMR1_Input = (9 << 12) | (9 << 4) | (2 << 8) | 1;	 //  fSAMPLING=fDTS/8, N=8
	TIM2.CCMR2_Input = 0;
	TIM2.CCER		 = TIM2_CCER_CC1E | TIM2_CCER_CC2E | TIM2_CCER_CC2P;  // 1,2 enabled, 2 inverted
	TIM2.CR1 |= TIM2_CR1_CEN;
	NVIC_EnableIRQ(TIM2_IRQn);


    // ADC
    adc12_common_ccr_set_ckmode(&ADC12_Common, 3); // 80Mhz / 4
    ADC12_Common.CCR |= ADC12_COMMON_CCR_VBATSEL | ADC12_COMMON_CCR_VSENSESEL | ADC12_COMMON_CCR_VREFEN;
    ADC.CR = 0;  // end deep power down
    ADC.CR = ADC_CR_ADVREGEN;
    delay(25); // DS11453 table 66
    ADC.CR |= ADC_CR_ADCAL; // start calibration
    while (ADC.CR & ADC_CR_ADCAL)
        __NOP();

    // configure to sample channel 9, 11, 17, 18 and 0
    adc_sqr1_set_l3(&ADC, 5);   // 5 samples
    adc_sqr1_set_sq1(&ADC, 9);  // channel 9, pin PA4 thermistor
    adc_sqr1_set_sq2(&ADC, 11); // channel 11, pin PA6 current sense
    adc_sqr1_set_sq3(&ADC, 17); // channel 17, internal temperature
    adc_sqr1_set_sq4(&ADC, 11); // channel 18, Vbat (should be = Vcc)
    adc_sqr2_set_sq5(&ADC, 0);  // channel 0   Vrefint
    adc_smpr1_set_smp1(&ADC, 3); // 24.5 adc clock cycles 
    adc_smpr1_set_smp2(&ADC, 3); // 24.5 adc clock cycles 
    adc_smpr1_set_smp3(&ADC, 3); // 24.5 adc clock cycles 
    adc_smpr1_set_smp4(&ADC, 3); // 24.5 adc clock cycles 
    adc_smpr1_set_smp5(&ADC, 3); // 24.5 adc clock cycles 

    adc_cfgr_set_exten(&ADC, 1); // trigger on rising edge
    adc_cfgr_set_extsel(&ADC, 13); // ext 13 is TIM6 TRGO (see below)
    ADC.CFGR|= ADC_CFGR_CONT; // 

    ADC.IER |= ADC_IER_EOCIE | ADC_IER_EOSIE; // irq at end of conversion and end of sequence
    ADC.ISR |= ADC_ISR_ADRDY; // clear ready flag
    ADC.CR |= ADC_CR_ADEN;  // enable
    while (ADC.ISR & ADC_ISR_ADRDY)  // wait until ready
        __NOP();
    ADC.CR |= ADC_CR_ADSTART;  // start as soon as the triggers come in
//    NVIC_EnableIRQ(ADC1_IRQn);

	// set up TIM6 for a 8Hz hearbeat for various periodic things, including triggering the ADC
	// note: it pushes spiq messages so do not start this before the BMI/BME are configured.
	TIM6.DIER |= TIM6_DIER_UIE;
	TIM6.PSC = (CLOCKSPEED_HZ / 10000) - 1;
	TIM6.ARR = 1250 - 1;  // 10KHz/1250 = 8Hz
	TIM6.CR1 |= TIM6_CR1_CEN;
//    tim6_cr2_set_mms(&TIM6, 2); // TRGO = update event 
	NVIC_EnableIRQ(TIM6_DACUNDER_IRQn);

	// set up TIM16 for a 1Hz report 
	TIM16.DIER |= TIM16_DIER_UIE;
	TIM16.PSC = (CLOCKSPEED_HZ / 10000) - 1;
	TIM16.ARR = 10000 - 1;  // 10KHz/10000 = 1Hz
    TIM16.CNT = 3333;       // offset by a third of a second
	TIM16.CR1 |= TIM16_CR1_CEN;
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

	// headers used in output message vary by accel/gyro configuration
	gyro_hdr  = EVENTID_GYRO_2000DEG_S - gyro_cfg[0].val;  // minus is not a mistake
	accel_hdr = EVENTID_ACCEL_2G + accel_cfg[0].val;

	// BMI interrupt signals
    if (accel_ok) {
	    EXTI.IMR1  |= BMI_INT1A_PIN & Pin_All;
    	EXTI.FTSR1 |= BMI_INT1A_PIN & Pin_All;
    	NVIC_EnableIRQ(EXTI1_IRQn);	 // Accelerometer ready interrupt
    }
    if (gyro_ok) {
    	EXTI.IMR1  |= BMI_INT3G_PIN & Pin_All;
	    EXTI.FTSR1 |= BMI_INT3G_PIN & Pin_All;
    	NVIC_EnableIRQ(EXTI3_IRQn);	 // Gyroscope ready interrupt
    }

	// Initialize the independent watchdog
	IWDG.KR	 = 0x5555;	// enable watchdog config
	IWDG.PR	 = 0;		// prescaler /4 -> 10khz
	IWDG.RLR = 3200;	// count to 3200 -> 320ms timeout
	//IWDG.KR	 = 0xcccc;	// start watchdog countdown

	size_t packetsize = 960;  // 48 messages of 20 bytes
	size_t packetlen  = 0;
	size_t packetchk  = 0;

    printf("%lld mainloop start\n", cycleCount()/C_US);

	for (;;) {
        rt_start(&idle_rt, cycleCount());

		__WFI();

		struct SPIXmit *x	= spiq_tail(&spiq);
		struct Msg	   *ev	= msgq_tail(&evq);
        uint64_t        now = cycleCount();

        rt_stop(&idle_rt, now);

        if (!x && !ev)
            continue;

		// dequeue from spi and event queues, send on out queue
		// insert packet header and footer along the way
        rt_start(&mainloop_rt, now);
		while (x || ev) {
			struct Msg *out = msgq_head(&outq);
			if (!out) {
				++dropped_usart1;
				break;
			}

			if (ev) {
				out->len = ev->len;
				memmove(out->buf, ev->buf, ev->len);
				msgq_pop_tail(&evq);
                ev = msgq_tail(&evq);
			} else if (x) {
				int ok = output_bmx(out, x);
				spiq_deq_tail(&spiq);
                x = spiq_tail(&spiq);
    			if (!ok)  // nothing to send
	    			continue;
			}

			packetlen += out->len;
			for (size_t i = 0; i < out->len; ++i)
				packetchk += out->buf[i];

			msgq_push_head(&outq);
			USART1.CR1 |= USART1_CR1_TCIE;	// start USART1 if neccesary

			// not enough space for next message, pad with zeros
			if ((packetsize != packetlen) && (packetsize < packetlen + sizeof out->buf)) {
				while (msgq_head(&outq) == NULL)
					__NOP();
				out		 = msgq_head(&outq);
				out->len = packetsize - packetlen;
				bzero(out->buf, out->len);
				msgq_push_head(&outq);
				USART1.CR1 |= USART1_CR1_TCIE;	// start USART1 if neccesary

				packetlen = packetsize;
			}

            // end of packet; send checksum trailer + header for the next
			if (packetsize == packetlen) {
				while (msgq_head(&outq) == NULL)
					__NOP();
				out = msgq_head(&outq);
				msg_reset(out);
				msg_append16(out, packetchk);
				msg_append32(out, 0x49524F4E);	// 'IRON'
				msg_append16(out, packetsize);
				msgq_push_head(&outq);
				USART1.CR1 |= USART1_CR1_TCIE;	// start USART1 if neccesary

				packetchk = 0;
				packetlen = 0;

				// check here if there's a command response packet to send
			}

			IWDG.KR = 0xAAAA;  // pet the watchdog TODO check all subsystems

		}
        rt_stop(&mainloop_rt, cycleCount());

	} // forever

} // main()
