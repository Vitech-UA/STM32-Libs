/*
 * Gpio.h
 *
 *  Created on: 20 янв. 2021 г.
 *      Author: viktor.starovit
 */

#ifndef GPIO_H_
#define GPIO_H_

typedef enum GPIO_MODE {
		INPUT_MODE = 0, GP_OUTPUT_MODE, AF_MODE, ANALOG_MODE,
	} GPIO_MODE_t;

	typedef enum OT {
		OUTPUT_PP = 0, OUTPUT_OD
	} OUTPUT_TYPE_t;

	typedef enum OUT_SPEED {
		LOW_SPEED = 0, MEDIUM_SPEED, HIGH_SPEED
	} OUT_SPEED_t;

	typedef enum GPIO_PU_PD {
		NO_PUp_PDown = 0, PUp, PDown, RESERVED,
	} GPIO_PU_PD_t;

	typedef enum GPIO_AF {
		AF0 = 0, AF1, AF2, AF3, AF4, AF5, AF6, AF7,
	} GPIO_AF_t;

class Gpio {
public:
	Gpio(GPIO_MODE_t GPIO_MODE, OUT_SPEED_t GPIO_SPEED);
	//Gpio(GPIO_TypeDef *GPIO_PORT, uint16_t GPIO_PIN,GPIO_AF_t GPIO_ALTERNATE_FUNCTION, GPIO_MODE_t GPIO_MODE = AF_MODE);
private:

};

#endif /* GPIO_H_ */
