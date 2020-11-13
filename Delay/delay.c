/*
 * delay.c
 *
 *  Created on: 20 груд. 2017 р.
 *      Author: Andriy
 */

#include <delay.h>
#include "stm32f7xx.h"

// Функція вмикання таймеру для потреб delay
void delayInit(void)
{
#ifdef TIMER1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
#endif

#ifdef TIMER2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
#endif

#ifdef TIMER3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
#endif

#ifdef TIMER4
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
#endif
}

// Функція вимикання таймеру як немає потреби в delay
void delayDeInit(void)
{
#ifdef TIMER1
	RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
#endif

#ifdef TIMER2
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
#endif

#ifdef TIMER3
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
#endif

#ifdef TIMER4
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN;
#endif
}

//Функція формування затримки в мілісекундах
void delayMs(volatile uint32_t delay)
{
	CURRENT_TIMER->PSC = CURRENT_FREQ/1000-1; //Встановлюємо подрібнювач
	CURRENT_TIMER->ARR = delay; //встановлюємо значення переповнювання таймеру, а також і значення при якому генеруеться подія оновлення
	CURRENT_TIMER->EGR |= TIM_EGR_UG; //Генерируемо Подію оновлення для запису даних в регістри PSC і ARR
	CURRENT_TIMER->CR1 |= TIM_CR1_CEN|TIM_CR1_OPM; //Запускаемо таймер записом биту CEN і встановлюємо режим Одного проходу встановленням біту OPM
	while ((CURRENT_TIMER->CR1) & (TIM_CR1_CEN!=0)); //Виконуємо цикл поки рахує таймер до нуля
}

//Функція формування затримки в мікросекундах
void delayUs(volatile uint32_t delay)
{
    CURRENT_TIMER->PSC = CURRENT_FREQ/1000000-1; ///Встановлюємо подрібнювач
	CURRENT_TIMER->ARR = delay; //встановлюємо значення переповнювання таймеру, а також і значення при якому генеруеться подія оновлення
	CURRENT_TIMER->EGR |= TIM_EGR_UG; //Генерируемо Подію оновлення для запису даних в регістри PSC і ARR
	CURRENT_TIMER->CR1 |= TIM_CR1_CEN|TIM_CR1_OPM; //Запускаемо таймер записом биту CEN і встановлюємо режим Одного проходу встановленням біту OPM
	while ((CURRENT_TIMER->CR1) & (TIM_CR1_CEN!=0)); //Виконуємо цикл поки рахує таймер до нуля
}
