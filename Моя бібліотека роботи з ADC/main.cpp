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


	Adc CurrentSensor = Adc(IN1);
	Adc VoltageSensor = Adc(IN0);
	Adc TempSensor = Adc(IN16);

	uint16_t CurrentSensorResult = 0;
	uint16_t VoltageSensorResult = 0;
	uint16_t TemperatureSensorResult = 0;

	while (1) {

		//CurrentSensorResult = CurrentSensor.GetValue();
		//VoltageSensorResult = VoltageSensor.GetValue();
		TemperatureSensorResult = TempSensor.GetMcuTemperature();
	}
}

