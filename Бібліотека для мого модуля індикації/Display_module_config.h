/*
 * Display_module_config.h
 *
 *  Created on: 8 мар. 2020 г.
 *      Author: Embed Viktor
 */

#ifndef DISPLAY_MODULE_CONFIG_H_
#define DISPLAY_MODULE_CONFIG_H_


#define USE_LED 0

#define USE_BUZZER 0

#define USE_BUTTON 0

#define STM32_SERIES 3

#define SPI_ITEM hspi1

#define LOAD_PORT GPIOC
#define LOAD_PIN GPIO_PIN_3

#if(USE_BUTTON > 0)
#define BUTTON1_PORT GPIOC
#define BUTTON1_PIN GPIO_PIN_13

#define BUTTON2_PORT GPIOB
#define BUTTON2_PIN GPIO_PIN_5
#endif

#if(USE_BUZZER > 0)

#define BUZZER_PORT GPIOC
#define BUZZER_PIN GPIO_PIN_14
#endif

#if(USE_LED > 0)
#define LED1_PORT GPIOC
#define LED1_PIN GPIO_PIN_13

#define LED2_PORT GPIOB
#define LED2_PIN GPIO_PIN_5
#endif

#endif /* DISPLAY_MODULE_CONFIG_H_ */
