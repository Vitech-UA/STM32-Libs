/*
 * ControlModule.h
 *
 *  Created on: Jan 23, 2021
 *      Author: Embed Viktor
 */

#ifndef CONTROLMODULE_H_
#define CONTROLMODULE_H_

#include "stm32f051x8.h"
#include <stdbool.h>

typedef enum LED_STATE {
	LED_ON, LED_OFF
} LED_STATE_t;

typedef enum BUTTON_STATE {
	PRESSED, RELEASED

} BUTTON_STATE_t;

typedef enum BUZZER_STATE {
	ON, OFF

} BUZZER_STATE_t;

/* Розкоментувати серію стм 32 */
#define STM32Fx 0
//#define STM32Fx 1
//#define STM32Fx 3
//#define STM32Fx 4
//#define STM32Fx 7

void GpioEnableClk(GPIO_TypeDef *PORT);

class ControlModule {
public:
	ControlModule(SPI_TypeDef *SPI_Item);
	// MAX7219
	void InitMax7219(SPI_TypeDef *SPI_Item);
	void Send7219();
private:
};

class Led {
public:
	void Config(GPIO_TypeDef *Led_Port, uint16_t Led_Pin);
	void SetState(LED_STATE_t LedState);
private:
	GPIO_TypeDef *Port;
	uint16_t Pin;
};

class Button {
public:
	void Config(GPIO_TypeDef *ButtonPort, uint16_t ButtonPin);
	BUTTON_STATE_t GetState(void);
private:
	GPIO_TypeDef *Port;
	uint16_t Pin;

};

class Buzzer {
public:
	void Config(GPIO_TypeDef *ButtonPort, uint16_t ButtonPin);
	void SetState(BUZZER_STATE_t);
private:
	GPIO_TypeDef *Port;
	uint16_t Pin;

};

#endif /* CONTROLMODULE_H_ */
