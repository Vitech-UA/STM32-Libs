/*
 * ds18b20_hal.h
 *
 *  Created on: Apr 16, 2022
 *      Author: Viktor
 */

#ifndef DS18B20_HAL_H_
#define DS18B20_HAL_H_

#include "main.h"

#define PORT  GPIOB
#define PIN 12
#define TIMER TIM3
#define SKIP_ROM_ADR 0xCC
#define CONVERT_TEMP 0x44
#define READ_DATA_COMAND 0xBE

uint8_t ds_reset_pulse(uint16_t PinMask);
void ds_write_bit(uint8_t bit, uint16_t PinMask);
uint16_t ds_read_bit(uint16_t PinMask);
void ds_write_byte(uint8_t byte, uint16_t PinMask);
uint8_t ds_read_byte(uint16_t PinMask);
uint16_t ds_get_temperature(void);
#endif /* DS18B20_HAL_H_ */
