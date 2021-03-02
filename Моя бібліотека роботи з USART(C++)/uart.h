/*
 * uart.h
 *
 *  Created on: 21 янв. 2021 г.
 *      Author: viktor.starovit
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f0xx.h"
#include "stdarg.h"

#define HCLK 8000000U

#define USE_RINGBUFFER // Розкоментувати для підключення кільцевого буфера

#define UART_RING_BUFFER_SIZE 128  // Максимальний розмір прийомного буфера


/* Прототипи обробників переривань. Реалізацію необхідно дописати власноруч */
extern "C" void USART1_IRQHandler(void);
extern "C" void USART2_IRQHandler(void);

class Uart {

public:
	Uart(USART_TypeDef *UartPort, uint32_t UsartBrrValue);
	void SendByte(uint8_t ByteToTransmit);
	void SendString(char *str);
	void SendString(uint8_t* str);
	uint8_t ReceiveByte(void);
	void EnableTxInterrupt(void);
	void EnableRxInterrupt(void);
	void ResetRxCompleteFlag(void);
	void RingBufferClear();
	uint16_t GetRingBufferSize(void);
	uint8_t ReadRingBuffer(void);
	//RingBuffer
	volatile uint16_t rx_buffer_head = 0;
	volatile uint16_t rx_buffer_tail = 0;
	uint8_t rx_buffer[UART_RING_BUFFER_SIZE] = { 0, };
	void Printf(const char *fmt, ...);

private:
	void InitGpio(void);
	void EnableClock(USART_TypeDef *UartPort);
	void Init(void);
	USART_TypeDef *ItemUsart;
	uint32_t ItemUsartBrrValue;

};

#endif /* UART_H_ */
