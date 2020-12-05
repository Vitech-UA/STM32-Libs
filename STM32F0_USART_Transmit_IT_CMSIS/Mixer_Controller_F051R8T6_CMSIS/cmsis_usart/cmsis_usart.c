/*
 * cmsis_usart.c
 *
 *  Created on: Nov 17, 2020
 *      Author: viktor.starovit
 */


#include "cmsis_usart.h"


__INLINE void Configure_GPIO_USART1(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  /* GPIO configuration for USART1 signals */
  /* (1) Select AF mode (10) on PB6 and PB7 */
  /* (2) AF0 for USART1 signals */

  	//**Tx Pin Init**//

  	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;         // PB7/PB6 Tx/Rx clk enable

  	GPIOB->MODER |= GPIO_MODER_MODER7_1;       // 1:0 - Alternate Function
  	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_7;        // 0 - Push-Pull
  	GPIOB->PUPDR &= ~ GPIO_PUPDR_PUPDR7;       // Reset
  	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_0;       // Pull Up
  	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;  // High speed
  	GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL7_Msk;     // AF0 PB7

  	//**Rx Pin Init**//
  	GPIOB->MODER |= GPIO_MODER_MODER6_1;       // 1:0 - Alternate Function
  	//GPIOB->MODER &= ~GPIO_MODER_MODER6;
  	GPIOB->PUPDR &= ~ GPIO_PUPDR_PUPDR6;       // 0:0 No pullUp pullDown
  	GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL6_Msk;     // AF0 PB6
}


 __INLINE void Configure_USART1(void)
 {
   /* Enable the peripheral clock USART1 */
   RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

   /* Configure USART1 */
   /* (1) oversampling by 16, 9600 baud */
   /* (2) 8 data bit, 1 start bit, 1 stop bit, no parity */
   int UsartBrrValue = 0;

   UsartBrrValue = 80000 / 96;

   USART1->BRR = UsartBrrValue;
  // USART1->BRR = 80000 / 96; /* (1) */
   USART1->CR1 = USART_CR1_TE | USART_CR1_UE; /* (2) */

   /* polling idle frame Transmission */
   while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC)
   {
     /* add time out here for a robust application */
   }
   USART1->ICR |= USART_ICR_TCCF;/* clear TC flag */
   USART1->CR1 |= USART_CR1_TCIE;/* enable TC interrupt */

   /* Configure IT */
   /* (3) Set priority for USART1_IRQn */
   /* (4) Enable USART1_IRQn */
  NVIC_SetPriority(USART1_IRQn, 0); /* (3) */
  NVIC_EnableIRQ(USART1_IRQn); /* (4) */
 }

