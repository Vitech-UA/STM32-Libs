/*
 * timer.c
 *
 *  Created on: 1 февр. 2021 г.
 *      Author: viktor.starovit
 */

#include "pwm.h"
#include "Gpio.h"

Pwm::Pwm(TIM_TypeDef *Timer, TIM_CHANNEL_t Channel) {

	InitGpio(Timer, Channel);
	EnableTimerClock();
	InitTimer();
}
void Pwm::SetPWM(uint16_t PWM_VALUE) {

	uint16_t pwm_buffer = PWM_VALUE;


	// Перевірка на вихід за допустимі межі.
	if(pwm_buffer > DEFAULT_MAX_PWM)
		pwm_buffer = DEFAULT_MAX_PWM;
	if(pwm_buffer < 0)
		pwm_buffer = 0;


    // Завантажую у регістр таймера
	switch ((int) this->ItemChannel) {
	case TIM_PWM_CH1:
		this->ItemTimer->CCR1 = (uint16_t)pwm_buffer;
		break;
	case TIM_PWM_CH2:
		this->ItemTimer->CCR2 = (uint16_t)pwm_buffer;
		break;
	case TIM_PWM_CH3:
		this->ItemTimer->CCR3 = (uint16_t)pwm_buffer;
		break;
	case TIM_PWM_CH4:
		this->ItemTimer->CCR4 = (uint16_t)pwm_buffer;
		break;

	}
}

void Pwm::InitGpio(TIM_TypeDef *Timer, TIM_CHANNEL_t Channel) {
#if (STM32SERIES == 0)
	////////////////////// TIMER 1 /////////////////////////////
	if (Timer == TIM1) {
		switch ((int) Channel) {

		case TIM_PWM_CH1: {

			Gpio pwm = Gpio(GPIOA, 8);
			pwm.SetAsAF(AF2, OUTPUT_PP);
			this->ItemTimer = TIM1;
			this->ItemChannel = TIM_PWM_CH1;

		}
			break;
		case TIM_PWM_CH2: {
			Gpio pwm = Gpio(GPIOA, 9);
			pwm.SetAsAF(AF2, OUTPUT_PP);
			this->ItemTimer = TIM1;
			this->ItemChannel = TIM_PWM_CH2;
		}
			break;
		case TIM_PWM_CH3: {
			Gpio pwm = Gpio(GPIOA, 10);
			pwm.SetAsAF(AF2, OUTPUT_PP);
			this->ItemTimer = TIM1;
			this->ItemChannel = TIM_PWM_CH3;
		}
			break;
		case TIM_PWM_CH4: {
			Gpio pwm = Gpio(GPIOA, 11);
			pwm.SetAsAF(AF2, OUTPUT_PP);
			this->ItemTimer = TIM1;
			this->ItemChannel = TIM_PWM_CH4;
		}
			break;

		}
	}
	////////////////////// TIMER 2 /////////////////////////////
	if (Timer == TIM2) {
		switch ((int) Channel) {
		case TIM_PWM_CH1: {
#ifdef TIM2_CH1_PA0
			Gpio pwm = Gpio(GPIOA, 0);
			pwm.SetAsAF(AF2, OUTPUT_PP);
			this->ItemTimer = TIM2;
			this->ItemChannel = TIM_PWM_CH1;
#endif

#ifdef TIM2_CH1_PA15
			Gpio pwm = Gpio(GPIOA, 15);
			pwm.SetAsAF(AF2, OUTPUT_PP);
			this->ItemTimer = TIM2;
			this->ItemChannel = TIM_PWM_CH1;
#endif
		}
			break;
		case TIM_PWM_CH2: {
#ifdef TIM2_CH2_PA1
			Gpio pwm = Gpio(GPIOA, 1);
			pwm.SetAsAF(AF2, OUTPUT_PP);
			this->ItemTimer = TIM2;
			this->ItemChannel = TIM_PWM_CH2;
#endif

#ifdef TIM2_CH2_PB3
			Gpio pwm = Gpio(GPIOB, 3);
		    pwm.SetAsAF(AF2, OUTPUT_PP);
		    this->ItemTimer = TIM2;
		    this->ItemChannel = TIM_PWM_CH2;
#endif
		}
			break;
		case TIM_PWM_CH3: {
#ifdef TIM2_CH3_PA2
			Gpio pwm = Gpio(GPIOA, 2);
			pwm.SetAsAF(AF2, OUTPUT_PP);
			this->ItemTimer = TIM2;
			this->ItemChannel = TIM_PWM_CH3;
#endif

#ifdef TIM2_CH3_PB10
			Gpio pwm = Gpio(GPIOB, 10);
			pwm.SetAsAF(AF2, OUTPUT_PP);
			this->ItemTimer = TIM2;
			this->ItemChannel = TIM_PWM_CH3;
#endif
		}
			break;
		case TIM_PWM_CH4: {
#ifdef TIM2_CH4_PA3
			Gpio pwm = Gpio(GPIOA, 3);
			pwm.SetAsAF(AF2, OUTPUT_PP);
			this->ItemTimer = TIM2;
			this->ItemChannel = TIM_PWM_CH4;
#endif

#ifdef TIM2_CH4_PB11
			Gpio pwm = Gpio(GPIOB, 11);
			pwm.SetAsAF(AF2, OUTPUT_PP);
			this->ItemTimer = TIM2;
			this->ItemChannel = TIM_PWM_CH4;
#endif
		}
			break;
		}
	}

	if (Timer == TIM3) {
		switch ((int) Channel) {

		case TIM_PWM_CH1:
			break;
		case TIM_PWM_CH2:
			break;
		case TIM_PWM_CH3:
			break;
		case TIM_PWM_CH4:
			break;

		}
	}

	if (Timer == TIM6) {
		switch ((int) Channel) {

		case TIM_PWM_CH1:
			break;
		case TIM_PWM_CH2:
			break;
		case TIM_PWM_CH3:
			break;
		case TIM_PWM_CH4:
			break;

		}
	}
#endif

#if(STM32SERIES==1)

#endif

#if(STM32SERIES==2)

#endif
}
void Pwm::EnableTimerClock(void) {
#if (STM32SERIES == 0)
	if (this->ItemTimer == TIM1) {
		if (!(RCC->APB2ENR & RCC_APB2ENR_TIM1EN))
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	}
	if (this->ItemTimer == TIM2) {
		if (!(RCC->APB1ENR & RCC_APB1ENR_TIM2EN))
			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	}
	if (this->ItemTimer == TIM3) {
		if (!(RCC->APB1ENR & RCC_APB1ENR_TIM3EN))
			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	}
	if (this->ItemTimer == TIM6) {
		if (!(RCC->APB1ENR & RCC_APB1ENR_TIM6EN))
			RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	}
#endif
}
void Pwm::InitTimer(void) {
	this->ItemTimer->CR1 |= TIM_CR1_CEN;
	this->ItemTimer->SMCR &= ~ TIM_SMCR_SMS;     // вн. тактування
	this->ItemTimer->CR1 &= ~TIM_CR1_DIR;        // Up counter
	this->ItemTimer->CCER = 0;
	this->ItemTimer->ARR = DEFAULT_MAX_PWM;
	this->ItemTimer->PSC = 4800 - 1; // Для частоти шима 10 кГц, при частоті тактування 48 МГц
	this->ItemTimer->BDTR |= TIM_BDTR_MOE; // OC and OCN outputs are enabled if their respective enable bits are set (CCxE, CCxNE in
										   // TIMx_CCER register).
	switch ((int) this->ItemChannel) {
	case TIM_PWM_CH1:
		this->ItemTimer->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 1
		this->ItemTimer->CCER |= TIM_CCER_CC1E;     // Вмикаю прямий Шим канал
		this->ItemTimer->CCR1 = 0;
		break;
	case TIM_PWM_CH2:
		this->ItemTimer->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // PWM mode 1
		this->ItemTimer->CCER |= TIM_CCER_CC2E;     // Вмикаю прямий Шим канал
		this->ItemTimer->CCR2 = 0;
		break;
	case TIM_PWM_CH3:
		this->ItemTimer->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // PWM mode 1
		this->ItemTimer->CCER |= TIM_CCER_CC3E;     // Вмикаю прямий Шим канал
		this->ItemTimer->CCR3 = 0;
		break;
	case TIM_PWM_CH4:
		this->ItemTimer->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // PWM mode 1
		this->ItemTimer->CCER |= TIM_CCER_CC4E;     // Вмикаю прямий Шим канал
		this->ItemTimer->CCR4 = 0;
		break;
	default:
		break;
	}
}
