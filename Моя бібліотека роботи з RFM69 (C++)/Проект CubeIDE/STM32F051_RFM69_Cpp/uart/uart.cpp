/*
 * uart.cpp
 *
 *  Created on: 21 янв. 2021 г.
 *      Author: viktor.starovit
 */

#include "uart.h"
#include "Gpio.h" // Для доступу до функції класу Gpio



/* Ремапи для USART1 */
#define USART1_PB6Tx_PB7Rx
//#define USART1_PA9Tx_PA10Rx

/* Ремапи для USART2 */
#define USART2_PA2Tx_PA3Rx
//#define USART2_PA14Tx_PA15Rx

/* Підтримка кільцевого буфера */
//#define UART_RX_BUFFER_SIZE 128 // Вказати максимальний розмір прийомного буфера


Uart::Uart(USART_TypeDef *UartPort, uint32_t UsartBrrValue) {
	this->ItemUsart = UartPort;
	this->ItemUsartBrrValue = UsartBrrValue;
	this->InitGpio();
	this->EnableClock(this->ItemUsart);
	this->Init();
}

#ifdef USE_RINGBUFFER

void Uart::RingBufferClear() {
	__NVIC_DisableIRQ(USART1_IRQn);
	this->rx_buffer_head = 0;
	this->rx_buffer_tail = 0;
	__NVIC_EnableIRQ(USART1_IRQn);
}

uint8_t Uart::ReadRingBuffer(void) {
	if (this->rx_buffer_head == this->rx_buffer_tail) // Якщо буфер пустий, то повертаємо ЗЕРО
			{
		return 0;
	} else {
		uint8_t c = rx_buffer[this->rx_buffer_tail];
		this->rx_buffer_tail =
				(uint16_t) (this->rx_buffer_tail + 1) % UART_RING_BUFFER_SIZE;
		return c;
	}
}

uint16_t Uart::GetRingBufferSize(void) {
	return ((uint16_t) (UART_RING_BUFFER_SIZE + this->rx_buffer_head - this->rx_buffer_tail))
			% UART_RING_BUFFER_SIZE;
}
#endif
void Uart::InitGpio(void) {
// Взалежності від того, який USART обрано його й ініціалізуємо
	if (this->ItemUsart == USART1) {

#ifdef USART1_PB6Tx_PB7Rx
		// PB6 USART1->Tx(AF0)
		// PB7 USART1->Rx(AF0)
		Gpio USART1_tx = Gpio(GPIOB, 6);
		Gpio USART1_rx = Gpio(GPIOB, 7);

		USART1_tx.SetAsAF(AF0, OUTPUT_PP);
		USART1_rx.SetAsAF(AF0);
#endif

#ifdef USART1_PA9Tx_PA10Rx

		// PA9 USART1->Tx(AF1)
		// PA10 USART1->Rx
		Gpio USART1_tx = Gpio(GPIOA, 9);
		Gpio USART1_rx = Gpio(GPIOA, 10);

		USART1_tx.SetAsAF(AF1, OUTPUT_PP);
		USART1_rx.SetAsAF(AF0);
#endif
	} else if (this->ItemUsart == USART2) {
#ifdef USART2_PA2Tx_PA3Rx
		// PA2 USART2->Tx(AF1)
		// PA3 USART2->Rx
		Gpio USART2_tx = Gpio(GPIOA, 2);
		Gpio USART2_rx = Gpio(GPIOA, 3);

		USART2_tx.SetAsAF(AF1, OUTPUT_PP);
		USART2_rx.SetAsAF(AF0);
#endif

#ifdef USART2_PA14Tx_PA15Rx
		// PA14 USART2->Tx(AF1)
		// PA15 USART2->Rx
		Gpio USART2_tx = Gpio(GPIOA, 14);
		Gpio USART2_rx = Gpio(GPIOA, 15);

		USART2_tx.SetAsAF(AF1, OUTPUT_PP);
		USART2_rx.SetAsAF(AF0);
#endif

	} else {
		// Обробка спроби ініціалізації неіснуючого USART порта
		// "ERROR: Unkown port"
	}
}

void Uart::EnableClock(USART_TypeDef *UartPort) {

	if (UartPort == USART1) {
		// Перед тим як ввімкнути тактування USART1 перевіримо, чи не було воно ввімкнене до цього?
		if (!(RCC->APB2ENR & RCC_APB2ENR_USART1EN)) {
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		}
	}
// Перед тим як ввімкнути тактування USART2 перевіримо, чи не було воно ввімкнене до цього?
	if (UartPort == USART2) {
		if (!(RCC->APB1ENR & RCC_APB1ENR_USART2EN)) {
			RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		}
	}

}

void Uart::Init(void) {
	this->ItemUsart->CR1 = 0;             // Скидаю налаштування
	this->ItemUsart->CR1 |= USART_CR1_UE; // Вмикаю модуль USART;

	//this->ItemUsart->BRR = 5000;          // Для 9600 Бод, при 48 МГц.

	if (this->ItemUsart->CR1 & USART_CR1_OVER8) {
		// Oversampling by 8

	} else {
		// Oversampling by 16
		int UsartBrrValue = 0;

		UsartBrrValue = HCLK / this->ItemUsartBrrValue;

		this->ItemUsart->BRR = UsartBrrValue;
	}

	this->ItemUsart->CR1 |= USART_CR1_TE; // Transmit enable
	this->ItemUsart->CR1 |= USART_CR1_RE; // Receive enable

	//this->ItemUsart->CR2 = 0;
	//this->ItemUsart->CR3 = 0;
}

uint8_t Uart::ReceiveByte(void) {

}

void Uart::SendByte(uint8_t ByteToTransmit) {
	while ((this->ItemUsart->ISR & USART_ISR_TXE) == 0) {
	}
	this->ItemUsart->TDR = ByteToTransmit;
}

void Uart::SendString(char *StringToTransmit) {

	uint8_t i = 0;
	while (StringToTransmit[i])
		this->SendByte(StringToTransmit[i++]);

}
void Uart::SendString(uint8_t* str){
	uint8_t i = 0;
		while (str[i])
			this->SendByte(str[i++]);
}
void Uart::EnableTxInterrupt(void) {

	if (!(this->ItemUsart->CR1 & USART_CR1_TXEIE)) {
		this->ItemUsart->CR1 |= USART_CR1_TXEIE;
	}

	if (this->ItemUsart == USART1) {
		NVIC_SetPriority(USART1_IRQn, 0);
		NVIC_EnableIRQ(USART1_IRQn);
	}

	if (this->ItemUsart == USART2) {
		NVIC_SetPriority(USART2_IRQn, 0);
		NVIC_EnableIRQ(USART2_IRQn);
	}
}
void Uart::EnableRxInterrupt(void) {
	if (!(this->ItemUsart->CR1 & USART_CR1_RXNEIE)) {
		this->ItemUsart->CR1 |= USART_CR1_RXNEIE;
		__NVIC_EnableIRQ(USART1_IRQn);
	}

	if (this->ItemUsart == USART1) {
		NVIC_EnableIRQ(USART1_IRQn);
	}

	if (this->ItemUsart == USART2) {
		NVIC_EnableIRQ(USART2_IRQn);
	}
}

void Uart::ResetRxCompleteFlag(void) {

}
