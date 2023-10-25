snippets

enable usart rx

    USART1.CR1 |= USART1_CR1_RXNEIE | USART1_CR1_RE; // enable IRQ input



usart handle RXNE req

static int input_sync_state = 0; // 0..3 is waiting for I,R,O,N, 4..5 is the length, 6 is in dma phase
static uint16_t input_frame_len = 0;
static uint8_t input_frame_buf[64];



  if ((isr & USART1_ISR_RXNE) != 0) {
        uint8_t c = USART1.RDR; // clears the flag

        switch (input_sync_state) {
        case 0: case 1: case 2: case 3:
            if (c == "IRON"[input_sync_state]) {
                ++input_sync_state;
            } else {
                input_sync_state = 0;
                input_frame_len = 0;
            }
            break;
        case 4: case 5:
            ++input_sync_state;
            input_frame_len <<= 8;
            input_frame_len |= c;
            break;
        }

        if (input_sync_state == 6) {
            // switch to dma
            USART1.CR1 &= ~USART1_CR1_RXNEIE;
            USART1.CR3 |= USART1_CR3_DMAR;
            DMA2.CCR7 = 0;
            dma1_cselr_set_c7s(&DMA2, 2); // select usart1 for dma2 ch7
            DMA2.CPAR7 = (uintptr_t)&USART1.RDR;
            DMA2.CMAR7 = (uintptr_t)input_frame_buf;
            DMA2.CNDTR7 = input_frame_len + 2;
            if (sizeof input_frame_buf + 2 < input_frame_len) {
                DMA2.CCR7 = DMA1_CCR7_TEIE | DMA1_CCR7_TCIE | DMA1_CCR7_EN;           
            } else {
                DMA2.CCR7 = DMA1_CCR7_TEIE | DMA1_CCR7_TCIE | DMA1_CCR7_EN | DMA1_CCR7_MINC;           
            }

        }
    }

uint32_t dma2ch6err_cnt = 0;
void DMA2_CH6_Handler(void) {
    if (DMA2.ISR & DMA1_ISR_TEIF6)
        ++dma2ch6err_cnt;
    DMA2.IFCR = DMA2.ISR & 0x00f00000;
}

    uint32_t dma2ch7err_cnt = 0;

    void DMA2_CH7_Handler(void) {
    if (DMA2.ISR & DMA1_ISR_TEIF7)
        ++dma2ch7err_cnt;
    DMA2.IFCR = DMA2.ISR & 0x0f000000;

    USART1.CR3 &= ~USART1_CR3_DMAR;
}
