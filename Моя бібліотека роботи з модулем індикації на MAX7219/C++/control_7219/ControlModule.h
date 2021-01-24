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

typedef enum AF {
	AF0,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,
	AF8,
	AF9,
	AF10,
	AF11,
	AF12,
	AF13,
	AF14,
	AF15
} AF_t;

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

/*Функції-утиліти*/
void GpioEnableClk(GPIO_TypeDef *PORT);
void GpioSetAsAF(GPIO_TypeDef *PORT, uint16_t gpio_pin, AF_t AF);

/*Інтерфейс бібліотеки*/
class ControlModule {
public:
	ControlModule();
	void Init();
	void Clear(void);
	void Print(volatile uint16_t number);
private:
	void nCS_LOW(void);
	void nCS_HIGH(void);
	void Transmit(uint8_t rg, uint8_t dt);
	void ConfigMAX7219(SPI_TypeDef *SPI_PORT);
	void ConfigGPIO(GPIO_TypeDef *clk_port, uint16_t clk_pin,
			GPIO_TypeDef *din_port, uint16_t din_pin, GPIO_TypeDef *load_port,
			uint16_t load_pin);

	GPIO_TypeDef *CLK_PORT;
	uint16_t CLK_PIN;
	GPIO_TypeDef *DIN_PORT;
	uint16_t DIN_PIN;
	GPIO_TypeDef *LOAD_PORT;
	uint16_t LOAD_PIN;
    SPI_TypeDef *SPI_ITEM;
	uint8_t aTxBuf[1]={0};
	char dg=4;
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
