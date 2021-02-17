/*
 * Servo.cpp
 *
 *  Created on: 15 февр. 2021 г.
 *      Author: viktor.starovit
 */

#include "Servo.h"
#include "pwm.h"


#define USE_SOFT_START

Servo::Servo(TIM_TypeDef *Timer, TIM_CHANNEL_t Channel):Pwm(Timer, Channel)
{

}
void Servo::SetDegree(uint8_t Degree){

#ifndef USE_SOFT_START
	uint16_t UsRange = (SERVO_180_DEGREE_US - SERVO_0_DEGREE_US);
	float Multipler = (float)(UsRange)  / 180.0;

	uint16_t buffer = (uint16_t)(Degree * Multipler) + SERVO_0_DEGREE_US; //TODO Видалити в реліз-версії

	this->SetPWM(buffer);
#endif

#ifdef USE_SOFT_START



#endif
}

