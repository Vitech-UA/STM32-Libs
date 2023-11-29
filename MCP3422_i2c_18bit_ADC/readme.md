# MCP3422 STM32 HAL LIB  
Example usage:
```  
if (HAL_I2C_IsDeviceReady(&hi2c1, MCP3422_ADDRESS, 1, 10) == HAL_OK)
	{
		HAL_UART_Transmit(&huart2, "MCP3424 Ready\r", 14, 10);
	}


uint32_t mv = MCP3422_get_mv(MCP3422_CHANNEL_1,
		MCP3422_RESOLUTION_18BIT, MCP3422_GAIN_1,
		MCP3422_CONVERSION_MODE_CONTINUOUS);
UART_Transmit_int(&huart2, mv);
HAL_Delay(300);

	
```
Конфігурація АЦП:  
![2023-11-22_21-10-58](https://github.com/Vitech-UA/STM32-Libs/assets/74230330/a2969ea1-4c3f-4a4a-87d6-28d86bd2660d)
Вичитка даних з АЦП:  
![2023-11-22_21-12-26](https://github.com/Vitech-UA/STM32-Libs/assets/74230330/372720d9-62d6-4da6-9ffe-ceb4e2d4e01b)


