/*
 * ds3231.c
 *
 *  Created on: Feb 4, 2022
 *      Author: Vitech-UA
 */

#ifndef ds3231_C_
#define ds3231_C_
#include "ds3231.h"

extern I2C_HandleTypeDef hi2c1;
const char *days_table[8] = { "", "Monday", "Tuesday", "Wednesday", "Thursday",
		"Friday", "Saturday", "Sunday" };
const char *month_table[13] = { "", "January", "February", "March", "April",
		"May", "June", "July", "August", "September", "October", "November",
		"December" };

void ds3231_init() {
	ds3231_set_Alarm1(DS3231_ALARM_DISABLED);
	ds3231_set_Alarm2(DS3231_ALARM_DISABLED);
	ds3231_clear_Alarm1_flag();
	ds3231_clear_Alarm2_flag();
	ds3231_set_interrupt_mode(DS3231_ALARM_INTERRUPT);
}

void ds3231_set_bbsqw_state(ds3231_bbsqw_state_t state) {
	uint8_t cr_old = ds3231_read_register(DS3231_CONTROL_REGISTER);
	/*Коли DS3231 живиться від vcc - осцилятор завжди ввімкнено
	 * незалежно від стану біта EOSC  */
	if (state == BBSQW_ENABLED) {
		uint8_t cr_new = cr_old & 0b10111111; // Обнуляю 6-й біт
		ds3231_write_register(DS3231_CONTROL_REGISTER, cr_new);

	}
	if (state == BBSQW_DISABLED) {
		uint8_t cr_new = cr_old | 0b0100000; // Встановлюю 6-й біт
		ds3231_write_register(DS3231_CONTROL_REGISTER, cr_new);
	}
}

void ds3231_set_sqw_state(ds3231_sqw_state_t state) {
	uint8_t cr_old = ds3231_read_register(DS3231_CONTROL_REGISTER);
	uint8_t cr_new;
	if (state == SQW_ENABLED) {
		cr_new = cr_old & 0b11111011; // Обнуляю 2-й біт
		ds3231_write_register(DS3231_CONTROL_REGISTER, cr_new);

	}
	if (state == SQW_DISABLED) {
		cr_new = cr_old | 0b0000100; // Встановлюю 2-й біт
		ds3231_write_register(DS3231_CONTROL_REGISTER, cr_new);
	}
}

void ds3221_set_oscilator_state(ds3231_state_t state) {
	uint8_t cr_old = ds3231_read_register(DS3231_CONTROL_REGISTER);
	ds3231_write_register(DS3231_CONTROL_REGISTER,
			(cr_old & 0x7f) | ((!state & 0x01) << DS3231_EOSC));
}

uint8_t ds3231_read_register(uint8_t reg_addr) {
	uint8_t val;
	HAL_I2C_Master_Transmit(&hi2c1, DS3231_I2C_ADDR, &reg_addr, 1,
	DS3231_TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1, DS3231_I2C_ADDR, &val, 1, DS3231_TIMEOUT);
	return val;

}

void ds3231_write_register(uint8_t reg_addr, uint8_t reg_val) {
	uint8_t bytes[2] = { reg_addr, reg_val };
	HAL_I2C_Master_Transmit(&hi2c1, DS3231_I2C_ADDR, bytes, 2, DS3231_TIMEOUT);
}

void ds3231_set_sqw_freq(sqw_freq_t freq) {
	uint8_t cr_old = ds3231_read_register(DS3231_CONTROL_REGISTER);
	ds3231_write_register(DS3231_CONTROL_REGISTER,
			(cr_old & 0xe7) | ((freq & 0x03) << DS3231_RS1));
}

float ds3231_get_temperature(void) {
	uint8_t temp[2];
	HAL_I2C_Mem_Read(&hi2c1, DS3231_I2C_ADDR, 0x11, 1, temp, 2, 1000);
	return ((temp[0]) + (temp[1] >> 6) / 4.0);
}

// Конвертація десяткового значення у BCD
uint8_t ds3231_dec_to_BCD(int val) {
	return (uint8_t) ((val / 10 * 16) + (val % 10));
}
// // Конвертація BCD у десяткове значення
int ds3231_BCD_to_dec(uint8_t val) {
	return (int) ((val / 16 * 10) + (val % 16));
}

void ds3231_set_time(date_time_t time) {
	uint8_t set_time[7];
	set_time[DS3231_SECONDS_REGISTER] = ds3231_dec_to_BCD(time.sec);
	set_time[DS3231_MINUTES_REGISTER] = ds3231_dec_to_BCD(time.min);
	set_time[DS3231_HOURS_REGISTER] = ds3231_dec_to_BCD(time.hour);
	set_time[DS3231_DOW_REGISTER] = ds3231_dec_to_BCD(time.day_of_weak);
	set_time[DS3231_DATE_REGISTER] = ds3231_dec_to_BCD(time.date);
	set_time[DS3231_MONTH_REGISTER] = ds3231_dec_to_BCD(time.month);
	set_time[DS3231_YEAR_REGISTER] = ds3231_dec_to_BCD(time.year);
	HAL_I2C_Mem_Write(&hi2c1, DS3231_I2C_ADDR, 0x00, 1, set_time, 7,
	DS3231_TIMEOUT);
}

date_time_t ds3231_get_time(void) {
	date_time_t time;
	uint8_t get_time[7];
	HAL_I2C_Mem_Read(&hi2c1, DS3231_I2C_ADDR, 0x00, 1, get_time, 7,
	DS3231_TIMEOUT);
	time.sec = ds3231_BCD_to_dec(get_time[DS3231_SECONDS_REGISTER]);
	time.min = ds3231_BCD_to_dec(get_time[DS3231_MINUTES_REGISTER]);
	time.hour = ds3231_BCD_to_dec(get_time[DS3231_HOURS_REGISTER]);
	time.day_of_weak = ds3231_BCD_to_dec(get_time[DS3231_DOW_REGISTER]);
	time.date = ds3231_BCD_to_dec(get_time[DS3231_DATE_REGISTER]);
	time.month = ds3231_BCD_to_dec(get_time[DS3231_MONTH_REGISTER]);
	time.year = ds3231_BCD_to_dec(get_time[DS3231_YEAR_REGISTER]);
	return time;
}

void ds3231_clear_Alarm1_flag() {
	uint8_t status = ds3231_read_register(DS3231_CONTROL_STATUS_REGISTER)
			& 0xfe;
	ds3231_write_register(DS3231_CONTROL_STATUS_REGISTER,
			status & ~(0x01 << DS3231_A1F));
}
void ds3231_clear_Alarm2_flag() {
	uint8_t status = ds3231_read_register(DS3231_CONTROL_STATUS_REGISTER)
			& 0xfd;
	ds3231_write_register(DS3231_CONTROL_STATUS_REGISTER,
			status & ~(0x01 << DS3231_A2F));
}

void ds3231_set_interrupt_mode(DS3231_InterruptMode_t mode) {
	uint8_t control = ds3231_read_register(DS3231_CONTROL_REGISTER);
	ds3231_write_register(DS3231_CONTROL_REGISTER,
			(control & 0xfb) | ((mode & 0x01) << DS3231_INTCN));
}

void ds3231_set_Alarm1(DS3231_Alarm_State_t enable) {
	uint8_t control = ds3231_read_register(DS3231_CONTROL_REGISTER);
	ds3231_write_register(DS3231_CONTROL_REGISTER,
			(control & 0xfe) | ((enable & 0x01) << DS3231_A1IE));
	ds3231_set_interrupt_mode(DS3231_ALARM_INTERRUPT);
}

void ds3231_set_Alarm2(DS3231_Alarm_State_t enable) {
	uint8_t control = ds3231_read_register(DS3231_CONTROL_REGISTER);
	ds3231_write_register(DS3231_CONTROL_REGISTER,
			(control & 0xfd) | ((enable & 0x01) << DS3231_A2IE));
	ds3231_set_interrupt_mode(DS3231_ALARM_INTERRUPT);
}

void ds3231_SetAlarm1Second(uint8_t second) {
	uint8_t temp = ds3231_read_register(DS3231_A1_SECOND) & 0x80;
	uint8_t a1m1 = temp | (ds3231_dec_to_BCD(second) & 0x3f);
	ds3231_write_register(DS3231_A1_SECOND, a1m1);
}
void ds3231_SetAlarm1Minute(uint8_t minute) {
	uint8_t temp = ds3231_read_register(DS3231_A1_MINUTE) & 0x80;
	uint8_t a1m2 = temp | (ds3231_dec_to_BCD(minute) & 0x3f);
	ds3231_write_register(DS3231_A1_MINUTE, a1m2);
}
void ds3231_SetAlarm1Hour(uint8_t hour_24mode) {
	uint8_t temp = ds3231_read_register(DS3231_A1_HOUR) & 0x80;
	uint8_t a1m3 = temp | (ds3231_dec_to_BCD(hour_24mode) & 0x3f);
	ds3231_write_register(DS3231_A1_HOUR, a1m3);
}
void ds3231_SetAlarm1Date(uint8_t date) {
	uint8_t temp = ds3231_read_register(DS3231_A1_DATE) & 0x80;
	uint8_t a1m4 = temp | (ds3231_dec_to_BCD(date) & 0x3f);
	ds3231_write_register(DS3231_A1_DATE, a1m4);
}
void ds3231_SetAlarm1Day(uint8_t day) {
	uint8_t temp = ds3231_read_register(DS3231_A1_DATE) & 0x80;
	uint8_t a1m4 = temp | (0x01 << DS3231_DYDT)
			| (ds3231_dec_to_BCD(day) & 0x3f);
	ds3231_write_register(DS3231_A1_DATE, a1m4);
}
void ds3231_set_Alarm1_mode(DS3231_Alarm1Mode_t alarmMode) {
	uint8_t temp;
	temp = ds3231_read_register(DS3231_A1_SECOND) & 0x7f;
	ds3231_write_register(DS3231_A1_SECOND,
			temp | (((alarmMode >> 0) & 0x01) << DS3231_AXMY));
	temp = ds3231_read_register(DS3231_A1_MINUTE) & 0x7f;
	ds3231_write_register(DS3231_A1_MINUTE,
			temp | (((alarmMode >> 1) & 0x01) << DS3231_AXMY));
	temp = ds3231_read_register(DS3231_A1_HOUR) & 0x7f;
	ds3231_write_register(DS3231_A1_HOUR,
			temp | (((alarmMode >> 2) & 0x01) << DS3231_AXMY));
	temp = ds3231_read_register(DS3231_A1_DATE) & 0x7f;
	ds3231_write_register(DS3231_A1_DATE,
			temp | (((alarmMode >> 3) & 0x01) << DS3231_AXMY)
					| (alarmMode & 0x80));
}
void ds3231_SetAlarm2Mode(DS3231_Alarm2Mode_t alarmMode) {
	uint8_t temp;
	temp = ds3231_read_register(DS3231_A2_MINUTE) & 0x7f;
	ds3231_write_register(DS3231_A2_MINUTE,temp | (((alarmMode >> 0) & 0x01) << DS3231_AXMY));
	temp = ds3231_read_register(DS3231_A2_HOUR) & 0x7f;
	ds3231_write_register(DS3231_A2_HOUR,temp | (((alarmMode >> 1) & 0x01) << DS3231_AXMY));
	temp = ds3231_read_register(DS3231_A2_DATE) & 0x7f;
	ds3231_write_register(DS3231_A2_DATE,temp | (((alarmMode >> 2) & 0x01) << DS3231_AXMY)| (alarmMode & 0x80));
}
uint8_t ds3231_IsAlarm1Triggered() {
	return (ds3231_read_register(DS3231_REG_STATUS) >> DS3231_A1F) & 0x01;
}

void ds3231_SetAlarm2Minute(uint8_t minute) {
	uint8_t temp = ds3231_read_register(DS3231_A2_MINUTE) & 0x80;
	uint8_t a2m2 = temp | (ds3231_dec_to_BCD(minute) & 0x3f);
	ds3231_write_register(DS3231_A2_MINUTE, a2m2);
}
void ds3231_SetAlarm2Hour(uint8_t hour_24mode) {
	uint8_t temp = ds3231_read_register(DS3231_A2_HOUR) & 0x80;
	uint8_t a2m3 = temp | (ds3231_dec_to_BCD(hour_24mode) & 0x3f);
	ds3231_write_register(DS3231_A2_HOUR, a2m3);
}
void ds3231_SetAlarm2Date(uint8_t date) {
	uint8_t temp = ds3231_read_register(DS3231_A2_DATE) & 0x80;
	uint8_t a2m4 = temp | (ds3231_dec_to_BCD(date) & 0x3f);
	ds3231_write_register(DS3231_A2_DATE, a2m4);
}
void ds3231_SetAlarm2Day(uint8_t day) {
	uint8_t temp = ds3231_read_register(DS3231_A2_DATE) & 0x80;
	uint8_t a2m4 = temp | (0x01 << DS3231_DYDT)
			| (ds3231_dec_to_BCD(day) & 0x3f);
	ds3231_write_register(DS3231_A2_DATE, a2m4);
}

uint8_t ds3231_IsAlarm2Triggered() {
	return (ds3231_read_register(DS3231_REG_STATUS) >> DS3231_A2F) & 0x01;
}

#endif /* ds3231_C_ */
