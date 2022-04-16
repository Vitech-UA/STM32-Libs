/*
 * ds18b20_hal.c
 *
 *  Created on: Apr 16, 2022
 *      Author: Viktor
 */

#include "ds18b20_hal.h"

uint16_t temperature;
char znak = '-';
uint8_t ds_buff[9];


uint8_t ds_reset_pulse(uint16_t PinMask)
{
	uint16_t result;

	if ((PORT->IDR & PinMask) == 0)
		return 2;
	PORT->ODR &= ~PinMask;
	TIMER->CNT = 0;
	while (TIMER->CNT < 480)
	{
	};
	PORT->ODR |= PinMask;
	while (TIMER->CNT < 550)
	{
	};
	result = PORT->IDR & PinMask;
	while (TIMER->CNT < 960)
	{
	};
		return 1;
	return 0;
}

void ds_write_bit(uint8_t bit, uint16_t PinMask)
{
	TIMER->CNT = 0;
	PORT->ODR &= ~PinMask;
	while (TIMER->CNT < 2)
	{
	};
	if (bit)
		PORT->ODR |= PinMask;
	while (TIMER->CNT < 60)
	{
	};
	PORT->ODR |= PinMask;
}

uint16_t ds_read_bit(uint16_t PinMask)
{
	uint16_t result;

	TIMER->CNT = 0;
	PORT->ODR &= ~PinMask;
	while (TIMER->CNT < 2)
	{
	};
	PORT->ODR |= PinMask;
	while (TIMER->CNT < 15)
	{
	};
	result = PORT->IDR & PinMask;
	while (TIMER->CNT < 60)
	{
	};
	return result;
}

void ds_write_byte(uint8_t byte, uint16_t PinMask)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
		ds_write_bit(byte & (1 << i), PinMask);
}


uint8_t ds_read_byte(uint16_t PinMask)
{
	uint8_t i, result = 0;
	for (i = 0; i < 8; i++)
		if (ds_read_bit(PinMask))
			result |= 1 << i;
	return result;
}

uint16_t ds_get_temperature(void)
{
	ds_reset_pulse(1 << PIN);
	ds_write_byte(SKIP_ROM_ADR, 1 << PIN);
	ds_write_byte(CONVERT_TEMP, 1 << PIN);
	HAL_Delay(1000);
	ds_reset_pulse(1 << PIN);
	ds_write_byte(SKIP_ROM_ADR, 1 << PIN);
	ds_write_byte(READ_DATA_COMAND, 1 << PIN);
	for (int i = 0; i < 9; i++)
		ds_buff[i] = ds_read_byte(1 << PIN);

	temperature = ds_buff[1];
	temperature = temperature << 8;
	temperature |= ds_buff[0];
	temperature = temperature >> 4;
	if (temperature > 1000)
	{
		temperature = 4096 - temperature;
		znak = '-';
	}
	else
		znak = '+';
	return temperature;
}
