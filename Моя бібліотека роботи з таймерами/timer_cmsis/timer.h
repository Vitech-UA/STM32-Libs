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

#endif /* TIMER_H_ */
