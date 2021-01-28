/*
 * uart.h
 *
 *  Created on: 21 янв. 2021 г.
 *      Author: viktor.starovit
 */

#ifndef UART_H_
#define UART_H_

#define HCLK 48000000U

/* Прототипи обробників переривань. Реалізацію необхідно дописати власноруч */
extern "C" void USART1_IRQHandler(void);
extern "C" void USART2_IRQHandler(void);


class Uart {

public:
	Uart(USART_TypeDef *UartPort, uint32_t UsartBrrValue);
	void SendByte(uint8_t ByteToTransmit);
	void SendString(char *str);
	uint8_t ReceiveByte(void);
	void EnableTxInterrupt(void);
	void EnableRxInterrupt(void);
    void ResetRxCompleteFlag(void);

private:
	void InitGpio(void);
	void EnableClock(USART_TypeDef *UartPort);
	void Init(void);
	USART_TypeDef *ItemUsart;
	uint32_t ItemUsartBrrValue;
};

#endif /* UART_H_ */
