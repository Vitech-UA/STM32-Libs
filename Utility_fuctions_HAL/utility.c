/*
 * utility.c
 *
 *  Created on: 13 февр. 2022 г.
 *      Author: Vitech-UA
 */

#include "utility.h"

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

void print_binary(uint8_t size, void const *const ptr)
{
	char UART_BUFFER[50];
	unsigned char *b = (unsigned char*) ptr;
	unsigned char byte;
	int i, j;
	sprintf(UART_BUFFER, "0b");
	HAL_UART_Transmit(&huart2, (uint8_t*)UART_BUFFER, strlen(UART_BUFFER), HAL_MAX_DELAY);

	for (i = size - 1; i >= 0; i--)
	{
		for (j = 7; j >= 0; j--)
		{
			byte = (b[i] >> j) & 1;
			sprintf(UART_BUFFER, "%u", byte);
			HAL_UART_Transmit(&huart2, (uint8_t*)UART_BUFFER, strlen(UART_BUFFER),
			HAL_MAX_DELAY);
		}
	}
	sprintf(UART_BUFFER, "\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)UART_BUFFER, strlen(UART_BUFFER), HAL_MAX_DELAY);
}

void I2C_ScanBus(void)
{
	uint8_t i = 0;
	char UART_BUFFER[20] =
	{ };
	bool search_result = false;
	/* Scan only for 112 allowed addresses */
	for (i = 0x07; i < 0x78; i++)
	{
		if (HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 10, 100) == HAL_OK)
		{
			search_result = true;
			sprintf(UART_BUFFER, "Find: 0x%02X\r\n", i << 1);
			HAL_UART_Transmit(&huart2, (uint8_t*) UART_BUFFER,
					strlen(UART_BUFFER), 100);
		}

	}
	if (!search_result)
	{
		sprintf(UART_BUFFER, "Devices not found\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) UART_BUFFER, strlen(UART_BUFFER),
				100);
	}
}

void UART_Printf(const char *fmt, ...) {
	char buff[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buff, sizeof(buff), fmt, args);
	HAL_UART_Transmit(&huart2, (uint8_t*) buff, strlen(buff),
	HAL_MAX_DELAY);
	va_end(args);
}
