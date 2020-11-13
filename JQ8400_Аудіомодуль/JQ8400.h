/*
 * JQ8400.h
 *
 *  Created on: 25 апр. 2019 г.
 *      Author: viktor.starovit
 */
#ifndef JQ8400_H_
#define JQ8400_H_
#include <stdint.h>
#include "stm32f0xx_hal.h"


#define	BYTE_START		0xAA
#define BYTE_STOP		0xEF
#define VOLUME_SETINGS  0x13

#define CMD_PLAY		0x02
#define CMD_PAUSE		0x03

extern UART_HandleTypeDef huart2;

#define HAL_UART huart2 /*Тут вказати який саме юарт буде використовуватись*/


/*Дефайни аудіо треків*/

#define BOILER_ERROR 16





#endif /* JQ8400_H_ */
