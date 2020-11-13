/*
 * PZEM_004.h
 *
 *  Created on: Mar 24, 2020
 *      Author: Embed Viktor
 */

#ifndef PZEM_004_H_
#define PZEM_004_H_

#include <stm32f3xx_hal.h>
#include "stdio.h"
#include "main.h"
#include "stdbool.h"

#define REG_VOLTAGE     0x0000
#define REG_CURRENT_L   0x0001
#define REG_CURRENT_H   0X0002
#define REG_POWER_L     0x0003
#define REG_POWER_H     0x0004
#define REG_ENERGY_L    0x0005
#define REG_ENERGY_H    0x0006
#define REG_FREQUENCY   0x0007
#define REG_PF          0x0008
#define REG_ALARM       0x0009

#define CMD_RHR         0x03
#define CMD_RIR         0X04
#define CMD_WSR         0x06
#define CMD_CAL         0x41
#define CMD_REST        0x42

#define WREG_ALARM_THR   0x0001
#define WREG_ADDR        0x0002

#define UPDATE_TIME     200

#define RESPONSE_SIZE 32
#define READ_TIMEOUT 100

#define PZEM_DEFAULT_ADDR    0xF8

#define PZEM_BAUD_RATE 9600

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart2;

//#define DEBUG

#define PZEM_UART &huart3

bool PZEM004Tv30_sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check,
		uint16_t slave_addr);
void PZEM004Tv30_init(uint8_t addr);
void PZEM004Tv30_setCRC(uint8_t *buf, uint16_t len);
bool PZEM004Tv30_sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check, uint16_t slave_addr);
uint16_t PZEM004Tv30_CRC16(const uint8_t *data, uint16_t len);
uint16_t PZEM004Tv30_recieve(uint8_t *resp, uint16_t len);
bool PZEM004Tv30_setAddress(uint8_t addr);
bool PZEM004Tv30_updateValues();
float PZEM004Tv30_voltage();
float PZEM004Tv30_current();
float PZEM004Tv30_power();
float PZEM004Tv30_energy();
float PZEM004Tv30_frequency();
float PZEM004Tv30_pf();
bool PZEM004Tv30_getPowerAlarm();
bool PZEM004Tv30_setPowerAlarm(uint16_t watts);
bool PZEM004Tv30_resetEnergy();
#endif /* PZEM_004_H_ */
