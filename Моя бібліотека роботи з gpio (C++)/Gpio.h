/*

 #include <stm32f051x8.h>
 #include <sys/_stdint.h>

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
	LOW_SPEED = 0, MEDIUM_SPEED, NOTUSE, HIGH_SPEED
} OUT_SPEED_t;

typedef enum GPIO_PU_PD {
	NO_PUp_PDown = 0, PUp, PDown, RESERVED,
} GPIO_PU_PD_t;

typedef enum GPIO_AF {
	AF0 = 0, AF1, AF2, AF3, AF4, AF5, AF6, AF7,
} GPIO_AF_t;

class Gpio {
public:
	Gpio(GPIO_TypeDef *PORT, uint16_t gpio_pin);
	void SetAsAF(GPIO_AF_t AlernateFunction);
	void SetAsAF(GPIO_AF_t AlernateFunction, OUTPUT_TYPE_t GpioOutputType);
	void SetAsGenerapPurporseOutput(OUTPUT_TYPE_t GpioOutputType);
	void SetAsInput(GPIO_PU_PD_t GpioPullUp_PullDown);
	void Set(void);
	void Reset(void);
	void Toggle(void);
	void Deinit(void);
	bool IsSet(void);
private:
	GPIO_TypeDef *item_port;
	uint16_t item_pin;
	void GpioEnableClk();
	void GpioSetOutputType(OUTPUT_TYPE_t GpioOutputType);
	void GpioSetOutputSpeed(OUT_SPEED_t GpioOutputSpeed);

};

#endif /* GPIO_H_ */
