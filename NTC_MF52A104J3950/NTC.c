/*
 * NTC.c
 *
 *  Created on: 23 апр. 2020 г.
 *      Author: Embed Viktor
 */

#include "NTC.h"
#include <math.h>

float THERMISTOR_RESISTANCE = 0.0;

volatile int adc = 0;

float ntc_get_temp(int adc_raw_data)
{
 /*Функція приймає сире значення каналу АЦП,
   Повертає*/
	float tCelsium = 0.0;

	THERMISTOR_RESISTANCE = BALANCE_RESISTOR * ((MAX_ADC / adc_raw_data) - 1);

	tCelsium =
			(BETA * ROOM_TEMP)
					/ (BETA
							+ (ROOM_TEMP
									* log10(
											THERMISTOR_RESISTANCE
													/ RESISTOR_ROOM_TEMP)))
					- 273.15;
	return tCelsium;
}

