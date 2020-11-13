/*
 * tlc5940_conf.h
 *
 *  Created on: Aug 21, 2020
 *      Author: viktor.starovit
 */

#ifndef TLC5940_CONF_H_
#define TLC5940_CONF_H_

#define MAX_PWM_VALUE 1000

#define BLANK_PORT GPIOC
#define BLANK_PIN GPIO_PIN_8

#define XLAT_PORT GPIOC
#define XLAT_PIN GPIO_PIN_9

#define TLC5940_PWM_TIMER htim2
#define TLC5940_SPI_PORT hspi1

/* Підключення TLC5940
 *
 *__STM32__________|__TLC5940__

 * PB3 (SPI1->SCK)  - SCLK(25)
 * PB5 (SPI1->MOSI) - SIN(26)
 * PC9              - XLAT(24)
 * PC8              - BLANK(23)
 * PA3(TIM2->CH4)   - GSCLK(18)
 * GND              - VPRG(27)
 * VCC              - DCPRG(19)
 *
 *
 * 1) Налаштовуємо один таймер на генерацію шим. зі скважністю 50% і подаємо його на пін GSCLK
 *    + генерування переривань по співпадінню.
 * 2) Ще один таймер налаштовуємо на генерування переривання по співпадінню кожної 1 мС.
 *    Воно використовується для ф-ї затримки.
 *
 * 3) SPI налаштовуємо на ширину слов 8 біт + частота не більше 30 Мгц. (Хоча на 52 Мгц працювало).
 *
 */

#endif /* TLC5940_CONF_H_ */
