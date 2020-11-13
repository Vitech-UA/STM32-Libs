#ifndef MAX7219_H_
#define MAX7219_H_


#include "main.h"
#include "Display_module_config.h"


#if(STM32_SERIES == 0)
#include "stm32f0xx_hal.h"


#elif(STM32_SERIES == 1)
#include "stm32f1xx_hal.h"


#elif(STM32_SERIES == 3)
#include "stm32f3xx_hal.h"

#elif(STM32_SERIES == 4)
#include "stm32f4xx_hal.h"

#elif(STM32_SERIES > 4)
#error "UNKOWN_STM32_SERIES"
#endif





void Send_7219 (uint8_t rg, uint8_t dt);
void Clear_7219 (void);
void Init_7219 (void);
void MAX7219_Print_Number(volatile uint16_t number);
#endif /* MAX7219_H_ */
