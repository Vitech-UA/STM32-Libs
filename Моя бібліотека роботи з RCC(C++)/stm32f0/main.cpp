/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f051x8.h"
#include "RCC.h"
#include "Gpio.h"

using namespace std;

void del(int delay) {
	while (delay) {
		delay--;
	}
}

int main(void) {

	Rcc RCC_MODULE = Rcc(HCLK_48MHz); // Об'єкт класа Clock

	MCO MCO_Out; // Об'єкт класа MCO
	Gpio mco_pin = Gpio(GPIOA, 8);
	mco_pin.SetAsAF(AF0, OUTPUT_PP);
	MCO_Out.SetChanel(MCO_SystemClock);

	Gpio fan_pin = Gpio(GPIOC, 9);
	fan_pin.SetAsGenerapPurporseOutput(OUTPUT_PP);

while(1)
{
	fan_pin.Set();
		del(4800000);
		fan_pin.Reset();
		del(4800000);
}
}
