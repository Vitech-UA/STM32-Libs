/*
 * cmsis_SysTick.h
 *
 *  Created on: Dec 17, 2020
 *      Author: viktor.starovit
 */

#ifndef CMSIS_SYSTICK_H_
#define CMSIS_SYSTICK_H_

#include "stm32f3xx.h"
#define SYSCLOCK 24000000U


void SysTick_Handler(void);
void SysTickInitUs(uint32_t TimeBaseUs);
void SysTickInitHz(uint32_t TimeBaseHz);
void SysTickInit1000Ms(void);
#endif /* CMSIS_SYSTICK_H_ */
