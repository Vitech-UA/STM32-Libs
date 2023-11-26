#include "stm32f1xx_hal.h"

#define MCP3422_ADDRESS 0xD2

typedef enum {
    MCP3422_CHANNEL_1 = 0,
    MCP3422_CHANNEL_2 = 1,
} MCP3422_Channel;

typedef enum {
    MCP3422_RESOLUTION_12BIT = 0,
    MCP3422_RESOLUTION_14BIT = 1,
    MCP3422_RESOLUTION_16BIT = 2,
    MCP3422_RESOLUTION_18BIT = 3,
} MCP3422_Resolution;

typedef enum {
    MCP3422_GAIN_1 = 0,
    MCP3422_GAIN_2 = 1,
    MCP3422_GAIN_4 = 2,
    MCP3422_GAIN_8 = 3,
} MCP3422_Gain;

typedef enum {
    MCP3422_CONVERSION_MODE_CONTINUOUS = 0,
    MCP3422_CONVERSION_MODE_ONE_SHOT = 1,
} MCP3422_ConversionMode;

void MCP3422_config(MCP3422_Channel channel,
		MCP3422_Resolution resolution, MCP3422_Gain gain,
		MCP3422_ConversionMode conversionMode);
uint32_t MCP3422_Read_Raw();
float MCP3422_ConvertToVolts(uint32_t rawData);
void UART_TransmitFloat(UART_HandleTypeDef *huart, float value);
void MCP3422_WaitForConversion();
