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
#include "stm32f3xx.h"

#define SYSCLK 8000000UL
#define TIMER_FOR_DELAY TIM6 // Please check Init() funtion to en clk


void DelayUs(uint16_t Delay_Us);
void DelayMs(uint16_t Delay_Ms);
void DelayInit(void);

#endif /* TIMER_H_ */
