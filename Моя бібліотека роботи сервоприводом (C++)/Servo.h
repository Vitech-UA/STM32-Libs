/*
 * Servo.h
 *
 *  Created on: 15 февр. 2021 г.
 *      Author: viktor.starovit
 */

#ifndef __SERVO_H_
#define __SERVO_H_
#include "pwm.h"

/*Визначення команд серво-заслонки Початок*/

#define FIRST_SERVO_ID 0x30
#define VALVE_OPEN 0x61  //a
#define VALVE_CLOSE 0x62 //b
#define VALVE_SET 0x63   // c

/*Визначення команд серво-заслонки Завершення*/
#define SERVO_0_DEGREE_US 544
#define SERVO_180_DEGREE_US 2470



class Servo: private Pwm {
public:
	void SetDegree(uint8_t Degree);
	Servo(TIM_TypeDef *Timer, TIM_CHANNEL_t Channel);
private:
	uint8_t
};

#endif /* SERVO_H_ */
