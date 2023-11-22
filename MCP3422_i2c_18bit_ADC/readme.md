# MCP3422 STM32 HAL LIB  
Example usage:
```  
MCP3422_config(MCP3422_CHANNEL_1, MCP3422_RESOLUTION_18BIT,
MCP3422_GAIN_8, MCP3422_CONVERSION_MODE_ONE_SHOT);
HAL_Delay(500);
float voltage = MCP3422_read_voltage_float();
sprintf(UART_BUFFER, "Volt: %f V\n", voltage);
HAL_UART_Transmit(&huart2, (uint8_t*) UART_BUFFER, strlen(UART_BUFFER),
HAL_MAX_DELAY);
```
Конфігурація АЦП:  
![2023-11-22_21-10-58](https://github.com/Vitech-UA/STM32-Libs/assets/74230330/a2969ea1-4c3f-4a4a-87d6-28d86bd2660d)
Вичитка даних з АЦП:  
![2023-11-22_21-12-26](https://github.com/Vitech-UA/STM32-Libs/assets/74230330/372720d9-62d6-4da6-9ffe-ceb4e2d4e01b)


