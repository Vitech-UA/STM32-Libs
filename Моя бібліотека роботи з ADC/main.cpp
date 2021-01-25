#include "main.h"
#include <stdint.h>
#include "stm32f051x8.h"
#include "Gpio.h"
#include "RCC.h"
#include "adc.h"
#include "mixer_core.h"
using namespace std;

void del(int delay) {
	while (delay) {
		delay--;
	}
}

int main(void) {
	// Налаштування модуля RCC
	Clock RCC_B;

	// Налаштування MCO
	MCO MCO_Out;
	Gpio mco_pin = Gpio(GPIOA, 8);
	mco_pin.SetAsAF(AF0, OUTPUT_PP);
	MCO_Out.SetChanel(MCO_SystemClock);

	// Налаштування піна вентилятора: XP6
	Gpio MixerFan = Gpio(GPIOC, 9);
	InitFan(MixerFan);

	// Налаштування сенсора: XP8
	Gpio Sensor = Gpio(GPIOB, 0);
	Sensor.SetAsInput(PDown);

	// Налаштування сенсора: XP9
	//Gpio SecSensor = Gpio(GPIOC, 5);
	//Sensor.SetAsInput(PDown);



	Adc CurrentSensorCh = Adc(IN0);

	uint16_t ADC_Result;

	while (1) {

		ADC1->CR |= ADC_CR_ADSTART;

			while (!(ADC1->ISR & ADC_ISR_EOC))
			{

			}
			ADC_Result = ADC1->DR;
		EnableFan(MixerFan);

	}
}

