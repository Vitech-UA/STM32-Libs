/*
 * timer.c
 *
 *  Created on: Dec 12, 2020
 *      Author: Embed Viktor
 */

#include "timer.h"

void DelayUs(uint16_t Delay_Us) {

	uint16_t tim_psc_val = 0;
	tim_psc_val = SYSCLK / 1000000 - 1;
	TIMER_FOR_DELAY->PSC = tim_psc_val;
	TIMER_FOR_DELAY->SR = 0;             // Очистка флага Update event
	TIMER_FOR_DELAY->ARR = Delay_Us;         // Сюди записуємо бажану затримку
	TIMER_FOR_DELAY->CR1 |= TIM_CR1_CEN; // Вмикаємо таймер
	while (!(TIMER_FOR_DELAY->SR & TIM_SR_UIF))
		;
}

void DelayMs(uint16_t Delay_Ms) {
	uint16_t tim_psc_val = 0;
	tim_psc_val = SYSCLK / 1000 - 1;
	TIMER_FOR_DELAY->PSC = tim_psc_val;
	TIMER_FOR_DELAY->SR = 0;                 // Очистка флага Update event
	TIMER_FOR_DELAY->ARR = Delay_Ms;         // Сюди записуємо бажану затримку
	TIMER_FOR_DELAY->CR1 |= TIM_CR1_CEN;     // Вмикаємо таймер
	while (!(TIMER_FOR_DELAY->SR & TIM_SR_UIF))
		;
}

void DelayInit(void)
{
#ifdef TIM6
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

#endif

#ifdef TIM7
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

#endif

}
