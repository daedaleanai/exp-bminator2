#include "usart.h"
extern inline void usart_start(struct USART_Type* usart);
extern inline void usart_wait(struct USART_Type* usart);

void usart_init(struct USART1_Type* usart, int baud) {
	usart->CR1 = 0; 
	usart->CR2 = 0;
	usart->CR3 = 0;

    uint32_t clk = 80000000; // AHB1 = HCLCK 
 
	usart->BRR = (clk + baud/2) / baud;
	usart->CR1 |= USART1_CR1_UE | USART1_CR1_TE;
}

void usart_irq_handler(struct USART1_Type* usart, struct Ringbuffer* rb) {
	if (!ringbuffer_empty(rb)) {
		if ((usart->ISR & USART1_ISR_TXE) != 0) {
			usart->TDR = ringbuffer_get_tail(rb);   
		}
	} else {
		usart->CR1 &= ~USART1_CR1_TXEIE;
	}
	return;
}
