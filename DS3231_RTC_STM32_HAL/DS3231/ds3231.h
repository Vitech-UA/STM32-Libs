/*
 * ds3231.h
 *
 *  Created on: Feb 8, 2022
 *      Author: Vitech-UA
 */

#ifndef ds3231_H_
#define ds3231_H_
#include "stdint.h"
#include "stm32f3xx_hal.h"
#define DS3231_I2C_ADDR 	0xD0

#define DS3231_SECONDS_REGISTER 0x00
#define DS3231_MINUTES_REGISTER 0x01
#define DS3231_HOURS_REGISTER 0x02
#define DS3231_DOW_REGISTER 0x03
#define DS3231_DATE_REGISTER 0x04
#define DS3231_MONTH_REGISTER 0x05
#define DS3231_YEAR_REGISTER 0x06

#define DS3231_CONTROL_REGISTER 0x0E
#define DS3231_CONTROL_STATUS_REGISTER 0x0F

#define DS3231_A1_SECOND	0x07
#define DS3231_A1_MINUTE	0x08
#define DS3231_A1_HOUR		0x09
#define DS3231_A1_DATE		0x0a

#define DS3231_A2_MINUTE	0x0b
#define DS3231_A2_HOUR		0x0c
#define DS3231_A2_DATE		0x0d



#define DS3231_EOSC			7
#define DS3231_BBSQW		6
#define DS3231_CONV			5
#define DS3231_RS2			4
#define DS3231_RS1			3
#define DS3231_INTCN		2
#define DS3231_A2IE			1
#define DS3231_A1IE			0

#define DS3231_REG_STATUS	0x0f
#define DS3231_OSF			7
#define DS3231_EN32KHZ		3
#define DS3231_BSY			2
#define DS3231_A2F			1
#define DS3231_A1F			0
#define DS3231_AXMY			7
#define DS3231_DYDT			6
#define DS3231_TIMEOUT 1000

typedef enum {
	DS3231_STARTED, DS3231_STOPED
} ds3231_state_t;

typedef enum {
	SQW_1HZ = 0, SQW_1024HZ, SQW_4096HZ, SQW_8192HZ,
} sqw_freq_t;

typedef enum {
	SQW_DISABLED, SQW_ENABLED
} ds3231_sqw_state_t;

typedef enum {
	BBSQW_DISABLED, BBSQW_ENABLED
} ds3231_bbsqw_state_t;

typedef struct {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day_of_weak;
} date_time_t;

typedef enum{
	DS3231_SQUARE_WAVE_INTERRUPT, DS3231_ALARM_INTERRUPT
} DS3231_InterruptMode_t;

typedef enum{
	DS3231_ALARM_DISABLED, DS3231_ALARM_ENABLED
} DS3231_Alarm_State_t;

typedef enum{
	DS3231_A1_EVERY_S = 0x0f, DS3231_A1_MATCH_S = 0x0e, DS3231_A1_MATCH_S_M = 0x0c, DS3231_A1_MATCH_S_M_H = 0x08, DS3231_A1_MATCH_S_M_H_DATE = 0x00, DS3231_A1_MATCH_S_M_H_DAY = 0x80,
}DS3231_Alarm1Mode_t;

typedef enum {
	DS3231_A2_EVERY_M = 0x07, DS3231_A2_MATCH_M = 0x06, DS3231_A2_MATCH_M_H = 0x04, DS3231_A2_MATCH_M_H_DATE = 0x00, DS3231_A2_MATCH_M_H_DAY = 0x80,
}DS3231_Alarm2Mode_t;

void ds3231_init();
uint8_t ds3231_read_register(uint8_t reg_addr);
void ds3231_write_register(uint8_t reg_addr, uint8_t reg_val);
void ds3231_set_time(date_time_t time);
date_time_t ds3231_get_time(void);
void ds3221_set_oscilator_state(ds3231_state_t state);
void ds3231_set_sqw_state(ds3231_sqw_state_t state);
void ds3231_set_bbsqw_state(ds3231_bbsqw_state_t state);
void ds3231_set_sqw_freq(sqw_freq_t freq);
float ds3231_get_temperature(void);
void ds3231_clear_Alarm1_flag();
void ds3231_clear_Alarm2_flag();
void ds3231_set_interrupt_mode(DS3231_InterruptMode_t mode);
void ds3231_set_Alarm1(DS3231_Alarm_State_t enable);
void ds3231_set_Alarm2(DS3231_Alarm_State_t enable);
void ds3231_set_Alarm1_mode(DS3231_Alarm1Mode_t alarmMode);
void ds3231_SetAlarm1Second(uint8_t second);
void ds3231_SetAlarm1Minute(uint8_t minute);
void ds3231_SetAlarm1Hour(uint8_t hour_24mode);
void ds3231_SetAlarm1Date(uint8_t date);
void ds3231_SetAlarm1Day(uint8_t day);
uint8_t ds3231_IsAlarm1Triggered();
uint8_t ds3231_IsAlarm2Triggered();
void ds3231_SetAlarm2Mode(DS3231_Alarm2Mode_t alarmMode);
void ds3231_SetAlarm2Minute(uint8_t minute);
void ds3231_SetAlarm2Hour(uint8_t hour_24mode);
void ds3231_SetAlarm2Date(uint8_t date);
void ds3231_SetAlarm2Day(uint8_t day);

#endif /* ds3231_H_ */
