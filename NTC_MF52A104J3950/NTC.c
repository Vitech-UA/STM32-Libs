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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) //check if the interrupt comes from ACD1
	{
		adc = HAL_ADC_GetValue(&hadc1);
	}
}

double dNTCGetTemperature(int NTC_index) {
	double tCelsium = 0.0;

	HAL_ADC_Start(&hadc1); // Start conversion. Закоментувати якщо ввімкнено Continuos conversion mode
	HAL_ADC_PollForConversion(&hadc1, 100); // ожидаем окончания преобразования
	adc = HAL_ADC_GetValue(&hadc1); // читаем полученное значение в переменную adc
	HAL_ADC_Stop(&hadc1); // останавливаем АЦП (не обязательно)

	THERMISTOR_RESISTANCE = BALANCE_RESISTOR * ((MAX_ADC / adc) - 1);

	tCelsium =(BETA * ROOM_TEMP)/ (BETA+ (ROOM_TEMP* log10(THERMISTOR_RESISTANCE/ RESISTOR_ROOM_TEMP)))- 273.15;
	return tCelsium;
}

