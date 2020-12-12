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

void DelayInit(void) {
#ifdef TIM6
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

#endif

#ifdef TIM7
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

#endif

}
void PWM_Init(void) {

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Ініціалізація PA8, як канал 1 Таймера 1.
	GPIOA->MODER |= GPIO_MODER_MODER8_1;
	/*
	 00: Input mode (reset state)
	 01: General purpose output mode
	 10: Alternate function mode !!!
	 11: Analog mode
	 */
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
	/*
	 x0: Low speed
	 01: Medium speed
	 11: High speed !!!
	 */
	GPIOA->AFR[1] |= 0x00000006; /* AF6: TIM1_CH1*/

	// Ініціалізація таймера
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // En clk

	TIMER_FOR_PWM->PSC = 3; /*For PWM_FREQ = 2 kHz */

	TIMER_FOR_PWM->ARR = 1000; // мах pwm value

	// PWM_FREQ = SYSCLK / ((PSC+1) * (ARR + 1))

	TIMER_FOR_PWM->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_3;

	/*
	 0110: PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
	 else inactive. In downcounting, channel 1 is inactive (OC1REF=‘0’) as long as
	 TIMx_CNT>TIMx_CCR1 else active (OC1REF=’1’).
	 0111: PWM mode 2 - In upcounting, channel 1 is inactive as long as
	 TIMx_CNT<TIMx_CCR1 else active. In downcounting, channel 1 is active as long as
	 TIMx_CNT>TIMx_CCR1 else inactive.
	 */
	//TIMER_FOR_PWM->CCR1 = TIM1->ARR / 2;
	TIMER_FOR_PWM->CCER |= TIM_CCER_CC1E;
	/*
	 0: Capture mode disabled / OC1 is not active (see below)
	 1: Capture mode enabled / OC1 signal is output on the corresponding output pin
	 *
	 */

	TIMER_FOR_PWM->BDTR |= TIM_BDTR_MOE;
	/*
	 MOE: Main output enable
	 This bit is cleared asynchronously by hardware as soon as one of the break inputs is active
	 (BRK or BRK2). It is set by software or automatically depending on the AOE bit. It is acting
	 only on the channels which are configured in output.
	 0: In response to a break 2 event. OC and OCN outputs are disabled
	 In response to a break event or if MOE is written to 0: OC and OCN outputs are disabled
	 or forced to idle state depending on the OSSI bit.
	 1: OC and OCN outputs are enabled if their respective enable bits are set (CCxE, CCxNE in
	 TIMx_CCER register).
	 */
	TIMER_FOR_PWM->CR1 |= TIM_CR1_CEN;
	/*
	 CEN: Counter enable
	 0: Counter disabled
	 1: Counter enabled

	 */

}
void PWM_SetDutyCycle(uint16_t Duty) {
	uint16_t ARR = TIMER_FOR_PWM->ARR;
	if (Duty > ARR)
		Duty = ARR;

	TIMER_FOR_PWM->CCR1 = Duty;

}
