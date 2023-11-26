/* MCP3422.c
*
* Created on: Nov 22, 2023
* Author: Vitech-UA
*/
#include "MCP3422.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

void MCP3422_config(MCP3422_Channel channel, MCP3422_Resolution resolution,
		MCP3422_Gain gain, MCP3422_ConversionMode conversionMode) {
	uint8_t configByte = 0x80;  // 7 біт - початок перетворення
	configByte |= (channel << 5);
	configByte |= (conversionMode << 4);
	configByte |= (resolution << 2);
	configByte |= gain;

	HAL_I2C_Master_Transmit(&hi2c1, MCP3422_ADDRESS, &configByte, 1,
	HAL_MAX_DELAY);
}

float MCP3422_read_voltage_float(void) {
	uint8_t i2cData[3];
	HAL_I2C_Master_Receive(&hi2c1, MCP3422_ADDRESS, i2cData, 3,
	HAL_MAX_DELAY);

	uint32_t adcResult = ((i2cData[0] & 0x0F) << 16) | (i2cData[1] << 8)
			| i2cData[2];
	return (float) adcResult * 2.048 / (float) (1 << 17); // 2.048 - напруга по джерелу
}

uint32_t MCP3422_Read_Raw() {
    uint8_t data[3];
    HAL_I2C_Mem_Read(&hi2c1, MCP3422_ADDRESS, 0x00, 1, data, 3, HAL_MAX_DELAY);

    // Розпаковка даних (використовуємо тільки молодші 18 біт)
    uint32_t result = ((data[0] & 0x03) << 16) | (data[1] << 8) | data[2];

    return result;
}

float MCP3422_ConvertToVolts(uint32_t rawData) {
	// Розрахунок напруги В за формулою для MCP3422 (Vref = 2.048V)
	float voltage = (rawData * 2.048) / (float) (1 << 18);
	return voltage;
}

void MCP3422_WaitForConversion() {
	uint8_t configByte = 0x80 | 0x10;
	HAL_I2C_Master_Transmit(&hi2c1, MCP3422_ADDRESS, &configByte, 1,
	HAL_MAX_DELAY);
	do {
		HAL_I2C_Master_Receive(&hi2c1, MCP3422_ADDRESS, &configByte, 1,
		HAL_MAX_DELAY);
	} while ((configByte & 0x80) == 0);
}

void UART_TransmitFloat(UART_HandleTypeDef *huart, float value) {
	char buffer[50];
	sprintf(buffer, "%.4f V\r\n", value);
	HAL_UART_Transmit(huart, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
}
