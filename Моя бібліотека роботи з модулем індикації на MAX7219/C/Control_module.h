#ifndef CONTROL_MODULE_H_
#define CONTROL_MODULE_H_

#include "stm32f1xx_hal.h"
#include "main.h"
#include "math.h"

void MAX7219_write_register (uint8_t rg, uint8_t dt);
void MAX7219_clear (void);
void MAX7219_print_int(volatile uint16_t number);
void MAX7219_print_temperature(volatile uint16_t number);
void MAX7219_print_float(float number);
void MAX7219_set_brightness(uint8_t Intensity);
void MAX7219_print_char(uint8_t position, uint8_t symbol);
void MAX7219_init (void);
void LED1_set_state(bool state);
void LED2_set_state(bool state);
void Buzzer_beep(void);
#endif /* CONTROL_MODULE_H_ */
