#include "spi.h"

#include "cortex_m4.h"
#include "nvic.h"

#define __IO volatile
struct DMA_Channel {
    __IO uint32_t CCR;   // @8 channel x configuration register
	__IO uint32_t CNDTR; // @12 channel x number of data register
	__IO uint32_t CPAR; // @16 channel x peripheral address register
	__IO uint32_t CMAR; // @20 channel x memory address register
};
#undef __IO

// indexed by enum SPIQ_DMA dma
static struct {
    struct DMA_Channel* rx;
    struct DMA_Channel* tx;
} const dmach[4] = {
   { (struct DMA_Channel*)&DMA1.CCR2, (struct DMA_Channel*)&DMA1.CCR3 },
   { (struct DMA_Channel*)&DMA1.CCR4, (struct DMA_Channel*)&DMA1.CCR5 },
   { (struct DMA_Channel*)&DMA2.CCR1, (struct DMA_Channel*)&DMA2.CCR2 },
   { (struct DMA_Channel*)&DMA2.CCR3, (struct DMA_Channel*)&DMA2.CCR4 },
};

// indexed by enum SPIQ_DMA dma
static const enum IRQn_Type irqn[4] = { DMA1_CH2_IRQn, DMA1_CH4_IRQn, DMA2_CH1_IRQn, DMA2_CH3_IRQn };

#define NELEM(x) (sizeof(x) / sizeof(x[0]))

// head >= curr >= tail
// empty:  head == curr == tail == 0
// push one at xmit[0]: head == 1, curr == 0 (running), tail == 0 -> deq_empty because curr=tail
// xmit[0] done:   head == 1, curr = 1 (stop) (enq empty because head == curr), tail = 0: 1 available for deq
// head == tail + 7 -> full
static inline int enq_empty(struct SPIQ *q) { return q->head == q->curr; }  // no more to xmit
static inline int deq_empty(struct SPIQ *q) { return q->curr == q->tail; }  // no more to dequeue
static inline int enq_full(struct SPIQ *q) { return q->head == q->tail + NELEM(q->elem); }

void spiq_init(
        struct SPIQ *q,
        struct SPI1_Type *spi,
        uint8_t clock_div,
        enum SPIQ_DMA dma,
        spi_slave_select_func_t ss_func) {

    q->spi = spi;
    q->dma = dma;
    q->ss_func = ss_func;
    q->head = q->curr = q->tail = 0;

    spi->CR1 = 0;
    // 8 bit master mode
    spi->CR1 = SPI1_CR1_MSTR | ((clock_div & 0x7) << 3);
    spi->CR2 = SPI1_CR2_FRXTH | SPI1_CR2_RXDMAEN | SPI1_CR2_TXDMAEN;

    if (ss_func != NULL) {
        spi->CR1 |= SPI1_CR1_SSM | SPI1_CR1_SSI;
    } else {
        spi->CR2 |= SPI1_CR2_SSOE;
    }

    switch (dma) {
    case SPI1_DMA1_CH23:  // C2S=1 C3S=1
            dma1_cselr_set_c2s(&DMA1, 1);
            dma1_cselr_set_c3s(&DMA1, 1);
            break;
    case SPI2_DMA1_CH45:  // C4S=1 C5S=1
            dma1_cselr_set_c4s(&DMA1, 1);
            dma1_cselr_set_c5s(&DMA1, 1);
            break;
    case SPI3_DMA2_CH12:  // C1S=3 C2S=3
            dma1_cselr_set_c1s(&DMA2, 3);
            dma1_cselr_set_c2s(&DMA2, 3);
            break;
    case SPI1_DMA2_CH34:  // C3S=4 C4S=4
            dma1_cselr_set_c3s(&DMA2, 4);
            dma1_cselr_set_c4s(&DMA2, 4);
            break;
    }

    NVIC_EnableIRQ(irqn[q->dma]);
}

inline static void spidmamove(struct SPI1_Type *spi, struct DMA_Channel *dmach, uint8_t *mem, size_t len) {
    dmach->CCR = DMA1_CCR1_MINC;  // all defaults, disable, except the Memory Increment flag
    dmach->CNDTR = len;
    dmach->CPAR = (uint32_t)&spi->DR;
    dmach->CMAR = (uint32_t)mem;
}

static void startxmit(struct SPIQ *q) {
    struct SPIXmit *x = q->elem + (q->curr % NELEM(q->elem));

    if (q->ss_func != NULL) {
        q->ss_func(x->addr, 1);
    }

    spidmamove(q->spi, dmach[q->dma].rx, x->buf, x->len);
    dmach[q->dma].rx->CCR |= DMA1_CCR1_TEIE | DMA1_CCR1_TCIE | DMA1_CCR1_EN;

    spidmamove(q->spi, dmach[q->dma].tx, x->buf, x->len);
    dmach[q->dma].tx->CCR |= DMA1_CCR1_TEIE | DMA1_CCR1_DIR | DMA1_CCR1_EN;

    q->spi->CR1 |= SPI1_CR1_SPE;
}

// idle == not enabled. start_xmit sets enable, dma irq handler clears it.
static inline int spi_idle(struct SPI1_Type *spi) {
    return (spi->CR1 & SPI1_CR1_SPE) ? 0 : 1;
}

void spiq_enq_head(struct SPIQ *q) {
    q->head++;

    //NVIC_DisableIRQ(irqn[q->dma]);
    if (spi_idle(q->spi)) {
        startxmit(q);
    }
    //NVIC_EnableIRQ(irqn[q->dma]);

    return;
}

void spi_rx_dma_handler(struct SPIQ *q) {
    struct SPIXmit* x =  q->elem + (q->curr % NELEM(q->elem));
    x->status = q->spi->SR & 0xf0;
    q->spi->CR1 &= ~SPI1_CR1_SPE;
    if (q->ss_func != NULL) {
        q->ss_func(x->addr, 0);
    }
    q->curr++;
    if (q->head != q->curr) {
        startxmit(q);
    }
}


// spi_wait blocks until the spi's queue has at least one finished xmit to deq, or the queue is idle.
void spi_wait(struct SPIQ *q) {
   while (deq_empty(q)) {
        if (spi_idle(q->spi) && enq_empty(q))
            return;
        __WFI();
    }
}


uint16_t spiq_xmit(struct SPIQ *q, uint16_t addr, size_t len, uint8_t* buf) {
    struct SPIXmit *x = spiq_head(q);
    if (!x) {
        return 0xffff;
    }
    x->addr = addr;
    x->len = len;
    x->buf = buf;
    spiq_enq_head(q);
    spi_wait(q);
    if (x != spiq_tail(q)) {
        // someone else got our result, expect massive problems
        return 0xffff;
    }
    uint16_t r = x->status;
    spiq_deq_tail(q);
    return r;
}