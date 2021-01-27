/*
 * uart.h
 *
 *  Created on: 21 янв. 2021 г.
 *      Author: viktor.starovit
 */

#ifndef UART_H_
#define UART_H_


/* Прототипи обробників переривань. Реалізацію необхідно дописати власноруч */
extern "C" void USART1_IRQHandler(void);
extern "C" void USART2_IRQHandler(void);


class Uart {

public:
	Uart(USART_TypeDef *UartPort);
	void SendByte(uint8_t ByteToTransmit);
	void SendString(char *str);
	void EnableTxInterrupt(void);
	void EnableRxInterrupt(void);
    void ResetRxCompleteFlag(void);

private:
	void InitGpio(void);
	void EnableClock(USART_TypeDef *UartPort);
	void Init(void);
	USART_TypeDef *ItemUsart;
};

#endif /* UART_H_ */
