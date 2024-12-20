/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "cmsis_usart.h"
#include "stm32f0xx.h"

uint8_t send = 0;
const uint8_t stringtosend[] = "Mixer controller v2.0 (cmsis firmware)\n\r";

int main(void) {

	Configure_GPIO_USART1();
	Configure_USART1();

	USART1->TDR = stringtosend[send++]; /* Will inititiate TC if TXE */

	while (1) {

	}

}

void USART1_IRQHandler(void) {
	if ((USART1->ISR & USART_ISR_TC) == USART_ISR_TC) {
		if (send == sizeof(stringtosend)) // Якщо у буфері ще є дані - передаємо їх далі
		{
			send = 0;
			USART1->ICR |= USART_ICR_TCCF; /* Очистка флага закінчення передачі */

		}
		else
		{
			/* clear transfer complete flag and fill TDR with a new char */
			USART1->TDR = stringtosend[send++];
		}
	} else {
		NVIC_DisableIRQ(USART1_IRQn); /* Disable USART1_IRQn */
	}

}
