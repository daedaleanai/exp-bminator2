/*
  Main body of bminator v2 firmware.

  Theory of operation:
  - initialize pinout, irqs and peripherals
  - initialize and self-test BMI088 and BME380
  - set up irq handlers for accel, gyro, shutter, and periodic events
  - the accel and gyro irqs push SPI transactions on the spiq, which are handled by DMA
  - the other irqs push messages on the event queue evq
  - incoming packets on USART1 are parsed by the USART1 irq.  valid requests are pushed
	on the spiq, invalid ones on the cmdq.
  - the mainloop copies spiq, evq and cmdq to the USART1 outq, which is also handled by DMA
	separating out the normal measurements from the command responses.

*/
#include "cortex_m4.h"
#include "stm32l4xx.h"

#include <string.h>

#include "binary.h"
#include "bmi08x.h"
#include "bmxspi.h"
#include "clock.h"
#include "crc16.h"
#include "gpio2.h"
#include "input.h"
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
	BMI_INT3G_PIN	  = PA3,  // TODO: kaya: USART2_RX
	THERMISTOR_PIN	  = PA4,  // ADC1_IN9
	HEATER_EN_PIN	  = PA5,
	CURRENT_SENSE_PIN = PA6,  // ADC1_IN11
	USART1_TX_PIN	  = PA9,
	USART1_RX_PIN	  = PA10,
	// TODO: kaya PA11 is BMI_INT3G
	TIMEPULSE_PIN	  = PA15,

	BMI_CSB2G_PIN = PB0,
	BMI_CSB1A_PIN = PB1,
	SPI1_SCK_PIN  = PB3,
	SPI1_MISO_PIN = PB4,
	SPI1_MOSI_PIN = PB5,
	BME_CSB_PIN	  = PB6,
};

static struct gpio_config_t {
	enum GPIO_Pin  pins;
	enum GPIO_Conf mode;
} const pin_cfgs[] = {
		{USART1_TX_PIN | USART1_RX_PIN, GPIO_AF7_USART123 | GPIO_HIGH},
		{USART2_TX_PIN, GPIO_AF7_USART123 | GPIO_HIGH},
		{SPI1_MOSI_PIN | SPI1_SCK_PIN | SPI1_MISO_PIN, GPIO_AF5_SPI12 | GPIO_HIGH},
		{BMI_INT1A_PIN | BMI_INT3G_PIN, GPIO_INPUT},
		{BMI_CSB1A_PIN | BMI_CSB2G_PIN | BME_CSB_PIN, GPIO_OUTPUT | GPIO_FAST},
		{TIMEPULSE_PIN, GPIO_INPUT},
		{THERMISTOR_PIN | CURRENT_SENSE_PIN, GPIO_ANALOG},
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
		{SysTick_IRQn, PRIO(0, 0)},		   // cycleCount 24 bit overflow
		{DMA1_CH2_IRQn, PRIO(1, 0)},	   // SPI RX done DMA pushes evq
		{DMA1_CH3_IRQn, PRIO(1, 0)},	   // SPI TX done DMA
		{TIM6_DACUNDER_IRQn, PRIO(1, 0)},  // 8Hz periodic read BME shedules SPI xmit and push evq
		{EXTI1_IRQn, PRIO(1, 1)},		   // BMI088 Accel IRQ schedules SPI xmit
		{EXTI3_IRQn, PRIO(1, 1)},		   // BMI088 Gyro IRQ schedule SPI xmit
		{TIM7_IRQn, PRIO(1, 1)},		   // BME280 schedule SPI xmit
		{TIM2_IRQn, PRIO(1, 2)},		   // Shutter open/close timer, push evq
		{ADC1_IRQn, PRIO(1, 2)},		   // ADC conversions
		{DMA2_CH7_IRQn, PRIO(2, 0)},	   // USART1 RX DMA
		{DMA2_CH6_IRQn, PRIO(2, 1)},	   // USART1 TX DMA done
		{USART1_IRQn, PRIO(2, 2)},		   // USART1 TX schedule next DMA
		{USART2_IRQn, PRIO(3, 1)},		   // USART2 TX send next character
		{TIM1_UP_TIM16_IRQn, PRIO(3, 3)},  // TIM16 1Hz periodic debug report on usart2
		{None_IRQn, 0xff},				   // sentinel
};
#undef PRIO

// RunTimers keep track of how often and how long certain blocks of code run
// There's typically one for each IRQ handler, for the main loop and for the idle
// wait.  the TIM16 handler dumps a report every second.
static struct RunTimer spixmit_rt     = {"spiq_xmit", 0, 0, 0, 0, NULL};        // latency between enq and xmit done
static struct RunTimer spiqdeq_rt 	  = {"spiq_deq", 0, 0, 0, 0, &spixmit_rt};  // latency between enq and deq
static struct RunTimer spirxdma_rt	  = {"SPIRXDMA", 0, 0, 0, 0, &spiqdeq_rt};
static struct RunTimer periodic8hz_rt = {"8HZTICK", 0, 0, 0, 0, &spirxdma_rt};
static struct RunTimer accelirq_rt	  = {"ACCELIRQ", 0, 0, 0, 0, &periodic8hz_rt};
static struct RunTimer gyroirq_rt	  = {"GYROIRQ", 0, 0, 0, 0, &accelirq_rt};
static struct RunTimer shutterirq_rt  = {"SHUTTER", 0, 0, 0, 0, &gyroirq_rt};
static struct RunTimer usart1rxdma_rt = {"USART1RXDMA", 0, 0, 0, 0, &shutterirq_rt};
static struct RunTimer usart1txdma_rt = {"USART1TXDMA", 0, 0, 0, 0, &usart1rxdma_rt};
static struct RunTimer usart1irq_rt	  = {"USART1IRQ", 0, 0, 0, 0, &usart1txdma_rt};
static struct RunTimer adcirq_rt	  = {"ADCIRQ", 0, 0, 0, 0, &usart1irq_rt};
static struct RunTimer report_rt	  = {"REPORT", 0, 0, 0, 0, &adcirq_rt};
static struct RunTimer mainloop_rt	  = {"MAIN", 0, 0, 0, 0, &report_rt};
static struct RunTimer wait_rt		  = {"WAIT", 0, 0, 0, 0, &mainloop_rt};
static struct RunTimer idle_rt		  = {"IDLE", 0, 0, 0, 0, &wait_rt};
uint64_t			   lastreport	  = 0;

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

// USART1 is the output datastream and command input
static struct MsgQueue	 outq				= {0, 0, {}};
static volatile uint32_t dropped_outq		= 0;  // queue full, msqq_head returned NULL
static volatile uint32_t usart1txdmaerr_cnt = 0;
static volatile uint32_t usart1rxdmaerr_cnt = 0;

// Most events go to the event message queue, which the mainloop copies into the output stream.
static struct MsgQueue	 evq		 = {0, 0, {}};
static volatile uint32_t dropped_evq = 0;  // queue full, msqq_head returned NULL

// Command responses, errors and read/write spi results go to the cmdq
static struct MsgQueue	 cmdq		  = {0, 0, {}};
static volatile uint32_t dropped_cmdq = 0;	// queue full, msqq_head returned NULL

// SPI1 has the BMI088, BME280 ... connected to it.
// The gyro and accel irq and periodically TIM6 trigger a SPI transaction on the accel, gyro or humid
// sensor. The result of that transaction becomes available in the main loop.
static struct SPIQ		 spiq;
static volatile uint32_t dropped_spi1	  = 0;	// how often we tried to submit a spi1 xmit but the queue was full
static volatile uint32_t spi1rxdmaerr_cnt = 0;
static volatile uint32_t spi1txdmaerr_cnt = 0;

// spi1_ss is the callback that maps spi addresses {GYRO, ACCEL, HUMID} to signals on the CSBx pins.
static const enum GPIO_Pin spiaddr2pin[3] = {BMI_CSB2G_PIN, BMI_CSB1A_PIN, BME_CSB_PIN};

static void spi1_ss(uint16_t addr, int on) {
	if ((addr < GYRO) || (addr > HUMID)) {
		return;	 // TODO should assert and crash
	}
	enum GPIO_Pin pin = spiaddr2pin[addr - GYRO];
	if (on) {
		digitalLo(pin);
	} else {
		digitalHi(pin);
	}
}

// SPI1 RX DMA
void DMA1_CH2_Handler(void) {
	uint64_t now = cycleCount();
	uint32_t isr = DMA1.ISR;
	if (isr & DMA1_ISR_TEIF2) {
		++spi1rxdmaerr_cnt;
	}
	DMA1.IFCR = isr & 0x00000f0;  // clear pending flags

	// account for spiq time spent between enq and deq 
	rt_start(&spixmit_rt, spiq.elem[spiq.curr % 8].ts);
	rt_stop(&spixmit_rt, now);

	spi_rx_dma_handler(&spiq);	  // see spi.c

	rt_start(&spirxdma_rt, now);
	rt_stop(&spirxdma_rt, cycleCount());
}

// SPI1 TX DMA only used to count errors
void DMA1_CH3_Handler(void) {
	uint32_t isr = DMA1.ISR;
	if (isr & DMA1_ISR_TEIF3) {
		++spi1txdmaerr_cnt;
	}
	DMA1.IFCR = isr & 0x0000f00;  // clear pending flags
}

// lowest 7 bits of tagreg are the register we're reading or writing
static void start_spix(uint64_t now, uint16_t addr, uint32_t tagreg, size_t len) {
	struct SPIXmit *x = spiq_head(&spiq);
	if (!x) {
		++dropped_spi1;
		return;
	}

	x->buf[0] = tagreg;	 // lowest 8 bits: read flag 0x80 + register address
	memset(x->buf + 1, 0xff, len - 1);

	x->ts	  = now;
	x->tag	  = tagreg;
	x->addr	  = addr;
	x->status = -1;
	x->len	  = len;
	spiq_enq_head(&spiq);
}

void EXTI1_Handler(void) {
	uint64_t now = cycleCount();
	if ((EXTI.PR1 & Pin_1) == 0) {
		return;
	}
	EXTI.PR1 = Pin_1;
	start_spix(now, ACCEL, 0x80 | BMI08x_ACC_X_LSB, 20);
	rt_start(&accelirq_rt, now);
	rt_stop(&accelirq_rt, cycleCount());
}

void EXTI3_Handler(void) {
	uint64_t now = cycleCount();
	if ((EXTI.PR1 & Pin_3) == 0) {
		return;
	}
	EXTI.PR1 = Pin_3;
	start_spix(now, GYRO, 0x80 | BMI08x_RATE_X_LSB, 10);
	rt_start(&gyroirq_rt, now);
	rt_stop(&gyroirq_rt, cycleCount());
}

// the BME280 doesn't have an IRQ pin, we just kick it once a second
// we don't bother timing this one
void TIM7_Handler(void) {
	if ((TIM7.SR & TIM6_SR_UIF) == 0) {
		return;
	}
	TIM7.SR &= ~TIM6_SR_UIF;
	start_spix(cycleCount(), HUMID, 0x80 | BME280_DATA_REG, 9);
}

// The 4 sampled channels are, in order
//     0  Vrefint
//     9  pin PA4 thermistor
//    11  pin PA6 current sense
//    17  internal temperature
static volatile int		 adc_chan	= 0;
static volatile uint32_t adc_ovfl	= 0;
static volatile uint16_t adc_val[4] = {0, 0, 0, 0};
static volatile uint64_t adc_ts[4]	= {0, 0, 0, 0};
static volatile uint32_t adc_trig	= 0;
extern uint16_t			 TS_CAL1, TS_CAL2, VREFINT;	 // defined in .ld file

void ADC1_Handler(void) {
	uint64_t now = cycleCount();
	uint16_t isr = ADC.ISR;

	// printf("%lld c%d%s%s%s\n", now, adc_chan,
	//     (isr & ADC_ISR_EOC) ? " EOC" : "",
	//     (isr & ADC_ISR_EOS) ? " EOS" : "",
	//     (isr & ADC_ISR_OVR) ? " OVR" : ""
	// );

	if (isr & ADC_ISR_EOC) {
		adc_val[adc_chan] = ADC.DR;
		adc_ts[adc_chan]  = now;
		++adc_chan;
	}
	if (isr & ADC_ISR_EOS) {
		adc_chan = 0;
		ADC.ISR |= ADC_ISR_EOS;
		//  printf("adc %u %u %u %u\n", adc_val[0], adc_val[1], adc_val[2], adc_val[3]);
		if (adc_trig++ % 8 == 0) {
			dropped_evq += output_internaltemperature(&evq, adc_ts[0], adc_val[0], adc_val[3]);
		}
	}

	if (isr & ADC_ISR_OVR) {
		ADC.ISR |= ADC_ISR_OVR;
		++adc_ovfl;
	}

	if (adc_chan >= 4) {
		adc_chan = 0;
		++adc_ovfl;
	}

	rt_start(&adcirq_rt, now);
	rt_stop(&adcirq_rt, cycleCount());
}

extern uint32_t UNIQUE_DEVICE_ID[3];  // Section 47.1, defined in .ld file

static volatile uint32_t tim6_tick = 0;
// Timer 6: 8Hz: emit periodic messages.  (also triggers ADC conversions through TRGO)
void TIM6_DACUNDER_Handler(void) {
	uint64_t now = cycleCount();
	if ((TIM6.SR & TIM6_SR_UIF) == 0) {
		return;
	}
	TIM6.SR &= ~TIM6_SR_UIF;
	// printf("%lld TRIGGER\n", now);
	switch (tim6_tick++ & 7) {
	case 2:
		dropped_evq += output_periodic(&evq, EVENTID_ID0, now, __REVISION__, UNIQUE_DEVICE_ID[2]);
		break;
	case 3:
		dropped_evq += output_periodic(&evq, EVENTID_ID1, now, UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
		break;
	case 4:
		dropped_evq += output_humidity(&evq);  // from last baro read
		break;
	}

	rt_start(&periodic8hz_rt, now);
	rt_stop(&periodic8hz_rt, cycleCount());
}

static volatile uint64_t shutter_count = 0;	 // open on odd counts, close on even.
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

	if ((open_ts && close_ts) && (open_ts <= close_ts)) {
		shutter_count = (shutter_count + 1) | 1;  // next odd number.
		dropped_evq += output_shutter(&evq, EVENTID_SHUTTER_OPEN, open_ts, shutter_count);
		open_ts = 0;  // mark as not happened, or already sent
	}

	if (close_ts) {
		shutter_count = (shutter_count | 1) + 1;  // next even number.
		dropped_evq += output_shutter(&evq, EVENTID_SHUTTER_CLOSE, open_ts, shutter_count);
	}
	// if it was detected and we didn't already send it before
	if (open_ts) {
		shutter_count = (shutter_count + 1) | 1;  // next odd number.
		dropped_evq += output_shutter(&evq, EVENTID_SHUTTER_OPEN, open_ts, shutter_count);
	}

	rt_stop(&shutterirq_rt, cycleCount());
}

// USART1 TX: irq on dma error or transmission complete
void DMA2_CH6_Handler(void) {
	rt_start(&usart1txdma_rt, cycleCount());
	uint32_t isr = DMA2.ISR;
	DMA2.IFCR	 = isr & 0x00f00000;
	if (isr & DMA1_ISR_TEIF6) {
		++usart1txdmaerr_cnt;
	}

	if (isr & DMA1_ISR_TCIF6) {
		if (msgq_tail(&outq)) {
			msgq_pop_tail(&outq);
		}
	}

	rt_stop(&usart1txdma_rt, cycleCount());
}

void USART1_Handler(void) {
	// Transmission Complete
	if ((USART1.ISR & USART1_ISR_TC) == 0) {
		return;
	}
	rt_start(&usart1irq_rt, cycleCount());

	struct Msg *msg = msgq_tail(&outq);
	if (msg == NULL) {
		// queue empty, clear IRQ Enable
		USART1.CR1 &= ~USART1_CR1_TCIE;

	} else {
		// queue not empty, start a new transfer
		// TODO(lvd) it's not entirely clear to me why we must only clear TC flag in this case
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

// set up usart1 rx dma for reception of len characters to be appended to cmdbuf
// set CC7 to run at higher priority level compared to ch6 (CCR PL bits)
static void usart1_start_rx(size_t len) {
	DMA2.CCR7 = 0;
	dma1_cselr_set_c7s(&DMA2, 2);  // select usart1 for dma2 ch7
	DMA2.CPAR7 = (uintptr_t)&USART1.RDR;
	DMA2.CMAR7 = (uintptr_t)cmdbuf.buf + cmdbuf.head;
	cmdbuf.head += len;
	DMA2.CNDTR7 = len;
	DMA2.CCR7	= DMA1_CCR7_PL | DMA1_CCR7_TEIE | DMA1_CCR7_TCIE | DMA1_CCR7_EN | DMA1_CCR7_MINC;
}

// USART1 RX: irq on dma error or reception complete
void DMA2_CH7_Handler(void) {
	rt_start(&usart1rxdma_rt, cycleCount());
	uint32_t isr = DMA2.ISR;
	DMA2.IFCR	 = isr & 0x0f000000;  // clear all pending
	if (isr & DMA1_ISR_TEIF7) {
		++usart1rxdmaerr_cnt;
	}

	if (isr & DMA1_ISR_TCIF7) {
		size_t n = input_cmdrx(&cmdq, &spiq);
		usart1_start_rx(n);
	}

	rt_stop(&usart1rxdma_rt, cycleCount());
}

// wait_outq loops until outq has room for a new message to enqueue
static inline struct Msg *wait_outq(void) {
	struct Msg *m;
	while ((m = msgq_head(&outq)) == NULL) {
		rt_start(&wait_rt, cycleCount());
		__WFI();
		rt_stop(&wait_rt, cycleCount());
	}
	return m;
}
// little helper for the main loop
static inline void start_outq(void) {
	__DMB();						// make sure all writes are committed so the DMA sees them
	msgq_push_head(&outq);			// enqueue the message at the head
	USART1.CR1 |= USART1_CR1_TCIE;	// if necessary start USART1
}

// A 1Hz report on the queues and all the handlers
void TIM1_UP_TIM16_Handler(void) {
	uint64_t now = cycleCount();
	uint16_t sr	 = TIM16.SR;
	TIM16.SR &= ~sr;

	uint64_t us	 = now / C_US;	// microseconds
	uint32_t vdd = adc_val[0] ? ((3000UL * VREFINT) / adc_val[0]) : 0;

	printf("\e[Huptime %llu.%06llu  Vdd %lu mV\e[K\n", us / 1000000, us % 1000000, vdd);
	printf("enqueued spiq: %8lu evq:%8lu cmdq: %2lu outq: %8lu\e[K\n", spiq.head, evq.head, cmdq.head, outq.head);
	printf("dropped  spiq: %8lu evq:%8lu cmdq: %2lu outq: %8lu\e[K\n", dropped_spi1, dropped_evq, dropped_cmdq, dropped_outq);
	printf("spi1   err tx: %8lu  rx:%8lu\e[K\n", spi1txdmaerr_cnt, spi1rxdmaerr_cnt);
	printf("usart1 err tx: %8lu  rx:%8lu\e[K\n", usart1txdmaerr_cnt, usart1rxdmaerr_cnt);
	if (adc_ovfl) {
		printf("adc ovfl %lu\e[K\n", adc_ovfl);
	}
	rt_report(&idle_rt, &lastreport);
	printf("\e[K\n");
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
	RCC.APB1ENR1 |= RCC_APB1ENR1_USART2EN | RCC_APB1ENR1_TIM6EN | RCC_APB1ENR1_TIM7EN | RCC_APB1ENR1_TIM2EN;
	RCC.APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM16EN;

	for (const struct gpio_config_t *p = pin_cfgs; p->pins; ++p) {
		gpioConfig(p->pins, p->mode);
	}

	gpioLock(PAAll);
	gpioLock(PBAll);

	// deselect all (active low) chip select signals, then wiggle them
	// to force them from I2C into SPI mode
	digitalHi(BMI_CSB1A_PIN | BMI_CSB2G_PIN | BME_CSB_PIN);
	for (enum BMXFunction i = GYRO; i <= HUMID; ++i) {
		delay(15);
		spi1_ss(i, 1);
		delay(15);
		spi1_ss(i, 0);
	}

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
	printf("cal ts %u %u vref %u\n", TS_CAL1, TS_CAL2, VREFINT);
	usart_wait(&USART2);


	// Set up SPI1 for talking to all the connected chips.
	spiq_init(&spiq, &SPI1, SPI_5MHz, SPI1_DMA1_CH23, spi1_ss);
	NVIC_EnableIRQ(DMA1_CH3_IRQn);	// only used to count tx errors

	// test and config BMI088

	if (bmi_accel_poweron(&spiq) == 0) {
		printf("BMI088 Accel enabled.\n");
	} else {
		printf("BMI088 not found.\n");
	}

	bmi088_self_test(&spiq);
	usart_wait(&USART2);

	if (bmi_accel_poweron(&spiq) != 0) {
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
	int gyro_ok = (bmx_check_config(&spiq, GYRO, gyro_cfg) == 0);
	if (!gyro_ok) {
		printf("BMI Gyro not properly configured\n");
	}
	usart_wait(&USART2);

	// test and config BME280

	if (bme280_self_test(&spiq, &bmeParam) == 0) {
		printf("T:");
		for (size_t i = 1; i < 4; ++i) {
			printf(" %ld", bmeParam.T[i]);
		}
		printf("\n");

		printf("P:");
		for (size_t i = 1; i < 10; ++i) {
			printf(" %ld", bmeParam.P[i]);
		}
		printf("\n");

		printf("H:");
		for (size_t i = 1; i < 7; ++i) {
			printf(" %ld", bmeParam.H[i]);
		}
		printf("\n");
	}
	usart_wait(&USART2);

	if (bmx_config(&spiq, HUMID, humid_cfg) != 0) {
		printf("error configuring BME humidity sensor\n");
	}
	usart_wait(&USART2);

	int humid_ok = (bmx_check_config(&spiq, HUMID, humid_cfg) == 0);
	if (!humid_ok) {
		printf("BME humidity sensor not properly configured.\n");
	}
	usart_wait(&USART2);

	// Prepare USART1 for high speed DMA driven output of the measurement data
	USART1.CR1 = 0;
	USART1.CR2 = 0;
	USART1.CR3 = 0;
	USART1.BRR = ((80000000 + 921600 / 2) / 921600);
	USART1.CR3 = USART1_CR3_DMAT | USART1_CR3_DMAR;	 // enable DMA output and input
	usart1_start_rx(CMDMINSIZE);
	USART1.CR1 = USART1_CR1_UE | USART1_CR1_TE | USART1_CR1_RE;
	NVIC_EnableIRQ(DMA2_CH6_IRQn);	// reception
	NVIC_EnableIRQ(DMA2_CH7_IRQn);	// transmission
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
	rcc_ccipr_set_adcsel(&RCC, 3);					// System clock selected as ADCs clock  6.4.27
	adc12_common_ccr_set_ckmode(&ADC12_Common, 1);	// HCLK, 80MHz

	ADC.CR &= ~ADC_CR_DEEPPWD;	// end deep power down
	ADC.CR |= ADC_CR_ADVREGEN;	// enable voltage regulator
	delay(25);					// DS11453 table 66
	ADC.CR |= ADC_CR_ADCAL;		// start calibration
	while (ADC.CR & ADC_CR_ADCAL) {
		__NOP();
	}

	ADC.ISR |= ADC_ISR_ADRDY;		   // clear ready flag
	ADC.CR |= ADC_CR_ADEN;			   // enable
	while (ADC.ISR & ADC_ISR_ADRDY) {  // wait until ready
		__NOP();
	}

	// Enable VRef on chan 0 and Temp Sens on chan 17
	ADC12_Common.CCR |= ADC12_COMMON_CCR_VSENSESEL | ADC12_COMMON_CCR_VREFEN;

	// configure to sample channels 0,9,11,17
	adc_sqr1_set_l3(&ADC, 4 - 1);  // 4 samples
	adc_sqr1_set_sq1(&ADC, 0);	   // channel  0, Vrefint cf sec 16.4.34 p 448
	adc_sqr1_set_sq2(&ADC, 9);	   // channel  9, pin PA4 thermistor
	adc_sqr1_set_sq3(&ADC, 11);	   // channel 11, pin PA6 current sense
	adc_sqr1_set_sq4(&ADC, 17);	   // channel 17, internal temperature cf 16.4.32 p 444

	adc_smpr1_set_smp0(&ADC, 7);   // 640.5 adc clock cycles 8us
	adc_smpr1_set_smp9(&ADC, 5);   //  92.5 adc clock cycles 1.2us
	adc_smpr2_set_smp11(&ADC, 5);  //  92.5 adc clock cycles 1.2us
	adc_smpr2_set_smp17(&ADC, 7);  // 640.5 adc clock cycles 8us

	adc_cfgr_set_exten(&ADC, 1);	// trigger on rising edge
	adc_cfgr_set_extsel(&ADC, 13);	// ext 13 is TIM6 TRGO (see below)

	ADC.CFGR |= ADC_CFGR_AUTDLY;  // without this, more than 3 samples trigger an overflow error (this was hard to find)
	ADC.IER = ADC_IER_EOCIE;	  // irq at end of conversion. the last one will also have EOS bit set, no need for extra IRQ
	ADC.CR |= ADC_CR_ADSTART;	  // start as soon as the triggers come in from TIM6
	NVIC_EnableIRQ(ADC1_IRQn);

	// set up TIM6 for a 8Hz hearbeat for various periodic things, including triggering the ADC
	// note: it pushes spiq messages so do not start this before the BMI/BME are configured.
	TIM6.DIER |= TIM6_DIER_UIE;
	TIM6.PSC = (CLOCKSPEED_HZ / 10000) - 1;
	TIM6.ARR = 1250 - 1;		 // 10KHz/1250 = 8Hz
	tim6_cr2_set_mms(&TIM6, 2);	 // TRGO = update event
	TIM6.CR1 |= TIM6_CR1_CEN;
	NVIC_EnableIRQ(TIM6_DACUNDER_IRQn);

	TIM7.DIER |= TIM6_DIER_UIE;
	TIM7.PSC = (CLOCKSPEED_HZ / 10000) - 1;
	TIM7.ARR = 10000 - 1;  // 10KHz/10000 = 1Hz
	TIM7.CNT = 2 * 3333;   // offset by two thirds of a second
	TIM7.CR1 |= TIM6_CR1_CEN;
	NVIC_EnableIRQ(TIM7_IRQn);

	// set up TIM16 for a 1Hz report
	TIM16.DIER |= TIM16_DIER_UIE;
	TIM16.PSC = (CLOCKSPEED_HZ / 10000) - 1;
	TIM16.ARR = 10000 - 1;	// 10KHz/10000 = 1Hz
	TIM16.CNT = 3333;		// offset by a third of a second
	TIM16.CR1 |= TIM16_CR1_CEN;
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

	// headers used in output message vary by accel/gyro configuration
	gyro_hdr  = EVENTID_GYRO_2000DEG_S - gyro_cfg[0].val;  // minus is not a mistake
	accel_hdr = EVENTID_ACCEL_3G + accel_cfg[0].val;

	// BMI interrupt signals
	if (accel_ok) {
		EXTI.IMR1 |= BMI_INT1A_PIN & Pin_All;
		EXTI.FTSR1 |= BMI_INT1A_PIN & Pin_All;
		NVIC_EnableIRQ(EXTI1_IRQn);	 // Accelerometer ready interrupt
	}
	if (gyro_ok) {
		EXTI.IMR1 |= BMI_INT3G_PIN & Pin_All;
		EXTI.FTSR1 |= BMI_INT3G_PIN & Pin_All;
		NVIC_EnableIRQ(EXTI3_IRQn);	 // Gyroscope ready interrupt
	}

	// Initialize the independent watchdog
	IWDG.KR	 = 0x5555;	// enable watchdog config
	IWDG.PR	 = 0;		// prescaler 32KHz/4 -> 8khz
	IWDG.RLR = 128;		// count to 128 -> 1/(64Hz) timeout
	IWDG.KR	 = 0xcccc;	// start watchdog countdown

	enum { PACKETSIZE = 960 };	// 48 messages of 20 bytes.
	size_t	 packetlen   = 0;
	uint16_t packetcrc16 = 0;
	uint64_t now	     = cycleCount();

	printf("%lld mainloop start\n", now / C_US);

	// mainloop has 2 parts: send out the plain events in packets of
	// fixed size and in between those, send out cmd response packets
	for (;;) {
		// wait until we have something to poll from the high speed / low latency spiq
		// which should be at least every 270 microseconds (3600 Hz) or from the low speed evq
		rt_start(&idle_rt, now);

		__WFI();

		struct SPIXmit *x  = spiq_tail(&spiq);
		struct Msg	   *ev = msgq_tail(&evq);

		now = cycleCount();
		rt_stop(&idle_rt, now);

		if (!x && !ev) {
			continue;
		}

		rt_start(&mainloop_rt, now);

		// if the spi result is a cmd packet, set it aside in the cmdq
		while (x && (x->tag & 0xffffff00)) {
			// disable the reception irq so we have exclusive access to cmdq
			NVIC_DisableIRQ(DMA2_CH6_IRQn);	 // reception
			struct Msg *msg = msgq_head(&cmdq);
			if (!msg) {
				++dropped_cmdq;
			} else {
				output_bmx(msg, x);
				msgq_push_head(&cmdq);
			}
			NVIC_EnableIRQ(DMA2_CH6_IRQn);	// reception

			spiq_deq_tail(&spiq);
			x = spiq_tail(&spiq);
		}

		if (!x && !ev) {
			now = cycleCount();
			rt_stop(&mainloop_rt, now);
			continue;
		}

		if (packetlen == 0) {
			// send header
			struct Msg *out = wait_outq();
			msg_reset(out);
			msg_append16(out, packetcrc16);	// crc16 of previous packet
			msg_append32(out, 0x49524F4E);	// 'IRON'
			msg_append16(out, PACKETSIZE);	// of the next packet
			start_outq();

			packetcrc16 = 0;

			// at 3600 message per second, packets of 48 messages of 20 bytes each should happen at 75Hz
			// the watchdog will trigger if we drop below 64Hz, so we have time to insert a command response
			IWDG.KR = 0xAAAA;  // pet the watchdog 
		}

		struct Msg *out = wait_outq();

		if (x) {
			output_bmx(out, x);
			spiq_deq_tail(&spiq);

			// account for spiq time spent between enq and deq 
			rt_start(&spiqdeq_rt, x->ts);
			rt_stop(&spiqdeq_rt, cycleCount());

		} else if (ev) {
			out->len = ev->len;
			memmove(out->buf, ev->buf, ev->len);
			msgq_pop_tail(&evq);
		} else {
			printf("PANIC\n");
			for (;;)
				__NOP();
		}

		packetlen += out->len;
		packetcrc16 = crc16_update(packetcrc16, out->buf, out->len);

		start_outq();

		// not enough space for next message, pad with zeros
		// (this shouldn't' happen as long as all normal messages are 20 bytes
		// long and packetsize is a multiple)
		if ((packetlen != PACKETSIZE) && (packetlen + 20 > PACKETSIZE)) {
			out		 = wait_outq();
			out->len = PACKETSIZE - packetlen;
			bzero(out->buf, out->len);
			packetlen += out->len;
			// TODO check: adding zeros should not change crc16
			start_outq();
		}

		if (packetlen == PACKETSIZE) {
			packetlen = 0;	// cause footer & header to be sent next time round

			struct Msg *rsp = msgq_tail(&cmdq);
			if (rsp != NULL) {
				// send header
				out = wait_outq();
				msg_reset(out);
				msg_append16(out, packetcrc16);
				msg_append32(out, 0x49524F4E);	// 'IRON'
				msg_append16(out, 4 + rsp->len);
				msg_append32(out, 0x06060606);	// tagged command response
				packetcrc16 = crc16_update(0, out->buf + out->len - 4, 4);  // 0x06060606 that was sent out above

				start_outq();

				out = wait_outq();
				msg_reset(out);
				out->len = rsp->len;
				memmove(out->buf, rsp->buf, rsp->len);
				msgq_pop_tail(&cmdq);

				// the chk16 will be sent out with next footer/header
				packetcrc16 = crc16_update(packetcrc16, out->buf, out->len);

				start_outq();
			}
		}

		now = cycleCount();
		rt_stop(&mainloop_rt, now);

	}  // forever

}  // main()
