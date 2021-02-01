/*
 * pwm.h
 *
 *  Created on: 1 февр. 2021 г.
 *      Author: viktor.starovit
 */

#ifndef PWM_H_
#define PWM_H_

#define DEFAULT_MAX_PWM 999

#define STM32SERIES 0
//#define STM32SERIES 1
//#define STM32SERIES 3

//Ремапи ШИМ каналів для TIM1 (в TIM1 канали ШИМ не ремапляться)
#define TIM1_CH1_PA8
#define TIM1_CH2_PA9
#define TIM1_CH3_PA10
#define TIM1_CH4_PA11

//Ремапи ШИМ каналів для TIM2
#define TIM2_CH1_PA0
//#define TIM2_CH1_PA15

#define TIM2_CH2_PA1
//#define TIM2_CH2_PB3

#define TIM2_CH3_PA2
//#define TIM2_CH3_PB10

#define TIM2_CH4_PA3
//#define TIM2_CH4_PB11

//Ремапи ШИМ каналів для TIM3

//Ремапи ШИМ каналів для TIM4

typedef enum TIM_CHANNEL {

	TIM_PWM_CH1 = 0, TIM_PWM_CH2, TIM_PWM_CH3, TIM_PWM_CH4
} TIM_CHANNEL_t;

class Pwm {

public:
	Pwm(TIM_TypeDef *Timer, TIM_CHANNEL_t Channel);
	void SetPWM(uint16_t PWM_VALUE);
private:
	void InitGpio(TIM_TypeDef *Timer, TIM_CHANNEL_t Channel);
	void EnableTimerClock(void);
	void InitTimer(void);
	TIM_TypeDef *ItemTimer;
	TIM_CHANNEL_t ItemChannel;
};

#endif

