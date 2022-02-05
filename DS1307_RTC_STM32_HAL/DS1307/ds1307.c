/*
 * ds1307.c
 *
 *  Created on: Feb 4, 2022
 *      Author: Vitech-UA
 */

#ifndef DS1307_C_
#define DS1307_C_
#include "ds1307.h"

extern I2C_HandleTypeDef hi2c1;
const char *days_table[8] = {"", "Monday", "Tuesday", "Wednesday", "Thursday",
		"Friday", "Saturday", "Sunday" };
const char *month_table[13] = {"", "January", "February", "March", "April",
		"May", "June", "July", "August", "September", "October", "November", "December" };

void ds1307_init(ds1307_state_t state) {
	uint8_t ch = (state ? 1 << 7 : 0);
	uint8_t new_second_register = (ch
			| (ds1307_read_register(DS1307_SECONDS_REGISTER) & 0x7f));
	ds1307_write_register(DS1307_SECONDS_REGISTER, new_second_register);
	ds1307_set_sqw(SQW_ENABLED);
	ds1307_set_sqw_freq(SQW_1HZ);

}

ds1307_state_t ds1307_get_clock_status() {

	uint8_t reg_val = ds1307_read_register(DS1307_SECONDS_REGISTER);

	if (reg_val & 0x80)
		return DS1307_STOPED;
	return DS1307_STARTED;
}

uint8_t ds1307_read_register(uint8_t reg_addr) {
	uint8_t reg_val = 0;
	HAL_I2C_Master_Transmit(&hi2c1, DS1307_I2C_ADDR << 1, &reg_addr, 1,
	DS1307_TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1, DS1307_I2C_ADDR << 1, &reg_val, 1,
	DS1307_TIMEOUT);
	return reg_val;

}

void ds1307_set_sqw(ds1307_sqw_state_t mode) {
	uint8_t control_reg = ds1307_read_register(DS1307_CONTROL_REGISTER);
	uint8_t new_control_register = ((control_reg & ~(1 << 4))
			| ((mode & 1) << 4));
	ds1307_write_register(DS1307_CONTROL_REGISTER, new_control_register);
}

void ds1307_set_sqw_freq(sqw_freq_t freq) {
	uint8_t control_reg = ds1307_read_register(DS1307_CONTROL_REGISTER);
	uint8_t new_control_register = ((control_reg & ~0x03) | freq);
	ds1307_write_register(DS1307_CONTROL_REGISTER, new_control_register);

}

void ds1307_write_register(uint8_t reg_addr, uint8_t reg_val) {
	uint8_t bytes[2] = { reg_addr, reg_val };
	HAL_I2C_Master_Transmit(&hi2c1, DS1307_I2C_ADDR << 1, bytes, 2,
	DS1307_TIMEOUT);
}

uint8_t ds1307_get_second() {
	return ds1307_decode_BCD(
			ds1307_read_register(DS1307_SECONDS_REGISTER) & 0x7f); // всі біти крім 7-го

}

uint8_t ds1307_get_minute() {
	return ds1307_decode_BCD(ds1307_read_register(DS1307_MINUTES_REGISTER));

}

uint8_t ds1307_get_hour() {
	return ds1307_decode_BCD(ds1307_read_register(DS1307_HOURS_REGISTER) & 0x3f);
}

uint8_t ds1307_get_day_of_weak() {
	return ds1307_decode_BCD(ds1307_read_register(DS1307_DOW_REGISTER));
}

uint8_t ds1307_get_month() {
	return ds1307_decode_BCD(ds1307_read_register(DS1307_MONTH_REGISTER));
}

uint8_t ds1307_get_date() {
	// Повертає день місяця
	return ds1307_decode_BCD(ds1307_read_register(DS1307_DATE_REGISTER));
}

uint16_t ds1307_get_year() {
	return ds1307_decode_BCD(ds1307_read_register(DS1307_YEAR_REGISTER));
}

uint8_t ds1307_decode_BCD(uint8_t bin) {
	return (((bin & 0xf0) >> 4) * 10) + (bin & 0x0f);
}

uint8_t ds1307_encode_BCD(uint8_t dec) {
	return (dec % 10 + ((dec / 10) << 4));
}

void ds1307_set_date_time(date_time_t date_time) {
	ds1307_write_register(DS1307_SECONDS_REGISTER,
			ds1307_encode_BCD(date_time.sec));
	ds1307_write_register(DS1307_MINUTES_REGISTER,
			ds1307_encode_BCD(date_time.min));
	ds1307_write_register(DS1307_HOURS_REGISTER,
			ds1307_encode_BCD(date_time.hour));
	ds1307_write_register(DS1307_DATE_REGISTER,
			ds1307_encode_BCD(date_time.date));
	ds1307_write_register(DS1307_DOW_REGISTER,
			ds1307_encode_BCD(date_time.day_of_weak));
	ds1307_write_register(DS1307_MONTH_REGISTER,
			ds1307_encode_BCD(date_time.month));
	ds1307_write_register(DS1307_YEAR_REGISTER,
			ds1307_encode_BCD(date_time.year));
}
#endif /* DS1307_C_ */
