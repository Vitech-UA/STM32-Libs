/*
 * timer.h
 *
 *  Created on: Dec 12, 2020
 *      Author: Embed Viktor
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "main.h"
#include "stdint.h"

#define SYSCLK 8000000UL     // Please set really timer clk freq

#define TIM_DMABURSTLENGTH_3TRANSFERS   0x00000200U
#define TIM_DMABASE_ARR   0x0000000BU
#define TIM1_DMAR_ADDRESS ((uint32_t)0x40012C4C) /* TIM DMAR address for burst access*/
#define BUFFER_DATA_NUMBER ((uint32_t)9)


/* Функції для генерації затримок на таймері*/
#define TIMER_FOR_DELAY TIM6 // Please check Init() funtion to en clk
void DelayUs(uint16_t Delay_Us);
void DelayMs(uint16_t Delay_Ms);
void DelayInit(void);

/* Функції для генерації PWM на таймері*/
#define TIMER_FOR_PWM TIM1   // Please check PWM_Init() funtion to en clk
void PWM_Init(void);
void PWM_SetDutyCycle(uint16_t Duty);

/* Функції для генерації одиночних PWM імпульсів */
void OnePwmPulseModeInit(uint16_t PulseLengthUs, uint16_t PulseCount);
void Pulse(); // Функція для генерування Заданої порції однакових імпульсів

/* Функції для вивода шим-сигналу довільної форми з буфера aSRC_Buffer(timer.c) через DMA */
void TIM1_PWM_DMA_BRUST_Init(void);

/* Функції для ініціалізації таймера в роботу в режимі переривань Update Обробник->TIM2_IRQHandler */
void InitTimerForInterruptGenerationMs(uint16_t TimeBaseMs);
void InitTimerForInterruptGenerationUs(uint16_t TimeBaseUs);
void TIM2_IRQHandler(void);
#endif /* TIMER_H_ */
