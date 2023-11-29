/* MCP3422.c
 *
 * Created on: Nov 22, 2023
 * Author: Vitech-UA
 */
#include "MCP3422.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

float scale[4] ={0.001, 0.000250, 0.0000625, 0.000015625};

void MCP3422_config(MCP3422_Channel channel, MCP3422_Resolution resolution,
		MCP3422_Gain gain, MCP3422_ConversionMode conversionMode)
{
	uint8_t configByte = 0x80;  // 7 біт - початок перетворення
	configByte |= (channel << 5);
	configByte |= (conversionMode << 4);
	configByte |= (resolution << 2);
	configByte |= gain;

	HAL_I2C_Master_Transmit(&hi2c1, MCP3422_ADDRESS, &configByte, 1,
	HAL_MAX_DELAY);
}

uint32_t MCP3422_get_raw(MCP3422_Channel channel, MCP3422_Resolution resolution,
		MCP3422_Gain gain, MCP3422_ConversionMode conversionMode)
{
	uint32_t result = 0;

	MCP3422_config(channel, resolution, gain, conversionMode);
	if (resolution == MCP3422_RESOLUTION_18BIT)
	{
		uint8_t data[4];
		do
		{

			HAL_I2C_Master_Receive(&hi2c1, MCP3422_ADDRESS, data, 4, 1000);
		} while ((data[3] & 0b10000000) == 0b10000000);
		result = (data[0] << 16) | (data[1] << 8) | (data[2]);
	}
	else
	{
		uint8_t data[3];
		do
		{
			HAL_I2C_Master_Receive(&hi2c1, MCP3422_ADDRESS, data, 3, 1000);
			result = (data[0] << 8) | data[1];
		} while ((data[2] & 0b10000000) == 0b10000000);
	}

	return result;
}

uint32_t MCP3422_get_mv(MCP3422_Channel channel, MCP3422_Resolution resolution,
		MCP3422_Gain gain, MCP3422_ConversionMode conversionMode)
{
	uint32_t result = 0;
	uint32_t data_mv = 0;

	MCP3422_config(channel, resolution, gain, conversionMode);
	if (resolution == MCP3422_RESOLUTION_18BIT)
	{
		uint8_t data[4];
		do
		{

			HAL_I2C_Master_Receive(&hi2c1, MCP3422_ADDRESS, data, 4, 1000);
		} while ((data[3] & 0b10000000) == 0b10000000);
		result = (data[0] << 16) | (data[1] << 8) | (data[2]);
	}
	else
	{
		uint8_t data[3];
		do
		{
			HAL_I2C_Master_Receive(&hi2c1, MCP3422_ADDRESS, data, 3, 1000);
			result = (data[0] << 8) | data[1];
		} while ((data[2] & 0b10000000) == 0b10000000);
	}
	data_mv = (result * scale[resolution]) * 1000;
	return data_mv;
}

float MCP3422_convert_to_v(uint32_t rawData)
{
	// Розрахунок напруги В за формулою для MCP3422 (Vref = 2.048V)
	float voltage = (rawData * 2.048) / (float) (1 << 18);
	return voltage;
}

void UART_TransmitFloat(UART_HandleTypeDef *huart, float value) {
	char buffer[50]={};
	//sprintf(buffer, "%.4f V\r\n", value); need enable sprinf_format
	HAL_UART_Transmit(huart, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
}

void UART_Transmit_int(UART_HandleTypeDef *huart, uint32_t value)
{
	char buffer[50];
	sprintf(buffer, "Read: %lu\n", value);
	HAL_UART_Transmit(huart, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
}
