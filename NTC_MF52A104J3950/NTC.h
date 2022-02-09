/*
 * NTC.h
 *
 *  Created on: 23 апр. 2020 г.
 *      Author: Embed Viktor
 */

#ifndef NTC_H_
#define NTC_H_

#include "main.h"

extern ADC_HandleTypeDef hadc1;

#define BALANCE_RESISTOR  46800.0
#define MAX_ADC  4095.0
#define BETA  3950.0
#define ROOM_TEMP  298.15
#define RESISTOR_ROOM_TEMP  100000.0

float ntc_get_temp(int adc_raw_data);


#endif /* NTC_H_ */
