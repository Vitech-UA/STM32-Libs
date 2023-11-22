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
