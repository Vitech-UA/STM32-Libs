/*
 * cmsis_SysTick.c
 *
 *  Created on: Dec 17, 2020
 *      Author: viktor.starovit
 */

#include "cmsis_SysTick.h"



void SysTick_Handler(void) {
	  GPIOA->ODR ^= GPIO_BSRR_BS_5;

}


void SysTickInitHz(uint32_t TimeBaseHz){
	SysTick->LOAD = (SYSCLOCK/TimeBaseHz-1);
	SysTick->VAL = TimeBaseHz;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

void SysTickInit1000Ms(void){
	SysTick->LOAD = (SYSCLOCK/1000);
	SysTick->VAL = 0x000000;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

}

void SysTickInitUs(uint32_t TimeBaseMs){

	SysTick->LOAD = (SYSCLOCK/4166 -1);
	SysTick->VAL = 0x000000;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

}



















