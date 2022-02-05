/*
 * ds1307.h
 *
 *  Created on: Feb 4, 2022
 *      Author: Vitech-UA
 */

#ifndef DS1307_H_
#define DS1307_H_
#include "stdint.h"
#include "stm32f3xx_hal.h"
#define DS1307_I2C_ADDR 	0x68

#define DS1307_SECONDS_REGISTER 0x00
#define DS1307_MINUTES_REGISTER 0x01
#define DS1307_HOURS_REGISTER 0x02
#define DS1307_DOW_REGISTER 0x03
#define DS1307_DATE_REGISTER 0x04
#define DS1307_MONTH_REGISTER 0x05
#define DS1307_YEAR_REGISTER 0x06
#define DS1307_CONTROL_REGISTER 0x07
#define DS1307_REG_UTC_HR	0x08
#define DS1307_REG_UTC_MIN	0x09
#define DS1307_REG_CENT    	0x10

#define DS1307_TIMEOUT 1000

typedef enum {
	DS1307_STARTED, DS1307_STOPED
} ds1307_state_t;

typedef enum {
	SQW_1HZ = 0, SQW_4096HZ, SQW_8192HZ, QSW_32768HZ,
} sqw_freq_t;

typedef enum {
	SQW_DISABLED, SQW_ENABLED
} ds1307_sqw_state_t;

typedef struct {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t date;
	uint8_t month;
	uint16_t year;
	uint8_t day_of_weak;
} date_time_t;



void ds1307_init(ds1307_state_t state);
ds1307_state_t ds1307_get_clock_status();
void ds1307_write_register(uint8_t reg_addr, uint8_t reg_val);
uint8_t ds1307_read_register(uint8_t reg_addr);
uint8_t ds1307_get_second();
uint8_t ds1307_get_minute();
uint8_t ds1307_get_hour();
uint8_t ds1307_get_day_of_weak();
uint8_t ds1307_get_month();
uint8_t ds1307_get_date();
uint16_t ds1307_get_year();
void ds1307_set_date_time(date_time_t date_time);
void ds1307_set_sqw(ds1307_sqw_state_t mode);
void ds1307_set_sqw_freq(sqw_freq_t freq);
uint8_t ds1307_decode_BCD(uint8_t bin);

#endif /* DS1307_H_ */
