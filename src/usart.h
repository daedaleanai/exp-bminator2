#pragma once

#include "stdarg.h"
#include "stddef.h"

#include "stm32l4xx.h"
#include "ringbuffer.h"

// usart_init() initializes USART{1,2,3,4,5}.
//
// The bytes will transmitted be as 8 data bits, Even Parity bit, 1 stop bit (8E1) at the specified baud rate.
//
// Before calling usart_init, make sure to set up the GPIO pins: TX to AF_PP/10MHz. RX to IN FLOATING or Pull-up.
// and to enable the USART in the RCC register:	RCC->APBxENR |= RCC_APBxENR_USARTyEN;
inline void usart_init(struct USART1_Type* usart, int baud) {
	usart->CR1 = 0; 
	usart->CR2 = 0;
	usart->CR3 = 0;
	usart->BRR = (80000000 + baud/2) / baud;
	usart->CR1 |= USART1_CR1_UE | USART1_CR1_TE;
}


// Create IRQ Handlers for the usarts you use like so:
// 	void USARTx_IRQ_Handler(void) { usart_irq_andler(&USARTx, txbufx); }
inline void usart_irq_handler(struct USART1_Type* usart, struct Ringbuffer* rb) {
	if (!ringbuffer_empty(rb)) {
		if ((usart->ISR & USART1_ISR_TXE) != 0) {
			usart->TDR = ringbuffer_get_tail(rb);   
		}
	} else {
		usart->CR1 &= ~USART1_CR1_TXEIE;
	}
	return;
}

// start transmission
inline void usart_start(struct USART1_Type* usart) { usart->CR1 |= USART1_CR1_TXEIE;  }

inline void usart_wait(struct USART1_Type* usart) {
	while (usart->CR1 & USART1_CR1_TXEIE)
		;
}