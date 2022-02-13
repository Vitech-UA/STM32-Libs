/*
 * utility.h
 *
 *  Created on: 13 февр. 2022 г.
 *      Author: Vitech-UA
 */

#ifndef UTILITY_H_
#define UTILITY_H_
#include "main.h"
#include "stdarg.h"

void print_binary(uint8_t size, void const * const ptr);
void I2C_ScanBus(void);
void UART_Printf(const char *fmt, ...);
#endif /* UTILITY_H_ */
