#pragma once

#include "stddef.h"
#include "stm32l4xx.h"

// A SPIXmit represents a transaction on a SPI device.
struct SPIXmit {
	uint64_t ts;   // timestamp
	uint32_t tag;  // tag to remember what this was for after xmit is done
	uint16_t addr;
	uint16_t status;
	size_t	 len;
	uint8_t  buf[32];
};

// The following SPI/DMA (rx/tx chan) combinations are available:
// cf RM0394 section 11.3 p298 table 14, 15
enum SPIQ_DMA {
	SPI1_DMA1_CH23,	 // C2S=1 C3S=1
	SPI2_DMA1_CH45,	 // C4S=1 C5S=1
	SPI3_DMA2_CH12,	 // C1S=3 C2S=3
	SPI1_DMA2_CH34	 // C3S=4 C4S=4
};

// This is the type of a function used instead of the NSS mechanism.
// If specified in spiq_init, on transmits this function is called with the
// address of the intended device to select/unselect based on the value of on.
// The address space is entirely up to the user.
typedef void spi_slave_select_func_t(uint16_t addr, int on);

// A SPIQ manages transactions on a SPI device.
// The user can push transactions into the queue which will be executed in the
// background after which te result can be popped off the queue again.
struct SPIQ {
	struct SPI1_Type		*spi;
	enum SPIQ_DMA			 dma;
	spi_slave_select_func_t *ss_func;

	// a ringbuffer with one element between head and tail
	// being currently transmitted by the SPI unit
	volatile uint32_t head;
	volatile uint32_t curr;
	volatile uint32_t tail;
	struct SPIXmit	  elem[8];	// must be power of 2 to make the indexing % size efficient.
};

// Initializes the SPIQ structure and sets up for use with spi as master,
// read/write, 8-bit transfers, pol/pha = 00 with the given clock divisor.
// valid values for div are 0...7 for divisor f/(2^(div+1))
// If ss_func is non-null, the NSS mechanism is disabled.
void spiq_init(struct SPIQ *q, struct SPI1_Type *spi, uint8_t clock_div, enum SPIQ_DMA dma, spi_slave_select_func_t ss_func);

// Call this function in the RX DMA handler as follows:
//
//    void DMAx_Channely_IRQ_Handler(void) {
//        DMAx.IFCR = DMAx.ISR & (0xf << 4*(y-1));  // chan 1: 0x000f, chan2
//        0x00f0 etc. spi_rx_dma_handler(spiqz);
//    }
//
// this will complete the transaction and start the next one on the queue.
void spi_rx_dma_handler(struct SPIQ *q);

#define NELEM(x) (sizeof(x) / sizeof(x[0]))

// spiq_head returns NULL if the queue is full,
// or the SPIXmit entry at the head of the queue for enqueuing.
inline struct SPIXmit *spiq_head(struct SPIQ *q) {
	return (q->head == q->tail + NELEM(q->elem)) ? NULL : &q->elem[q->head % NELEM(q->elem)];
}

// spiq_tail returns NULL if the queue is empty,
// or the SPIXmit entry at the tail of the queue for dequeuing.
inline struct SPIXmit *spiq_tail(struct SPIQ *q) { return (q->curr == q->tail) ? NULL : &q->elem[q->tail % NELEM(q->elem)]; }

#undef NELEM

// Enqueues the head  for transmission.
// Only call this if spiq_head() returned non-NULL, or the queue will overrun.
// The SPI will dma/irq the buffer out and in, after which the _tail function
// will return the result with the exchanged contents. The buffer contents
// should not be modified until this entry becomes available in the _tail
// function.
void spiq_enq_head(struct SPIQ *q);

// Marks the current tail entry as free to re-use.
inline void spiq_deq_tail(struct SPIQ *q) { ++q->tail; }

// spi_wait blocks until the spi's queue has at least one finished xmit to deq,
// or the queue is idle.
void spi_wait(struct SPIQ *q);

// utility for just doing a sync transaction.
// Do not use while async transactions are going on!
// returns the transaction status or 0xffff if something went wrong.
// in which case expect all SPI comms to be sufficiently messed up that you should reboot!
// you have been warned!
uint16_t spiq_xmit(struct SPIQ *q, uint16_t addr, size_t len, uint8_t *buf);