#include "AD9833.h"

extern SPI_HandleTypeDef hspi1;

uint16_t FRQLW = 0;
uint16_t FRQHW = 0;
uint32_t phaseVal = 0;
static uint16_t CONTROL_REGISTER;


void ad9833_write_16bit(uint16_t data)
{
	uint8_t reg[2];
	reg[0] = (data >> 8) & 0xFF;
	reg[1] = data & 0xFF;
	HAL_GPIO_WritePin(AD9833_CS_GPIO_Port, AD9833_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi1, &reg[0], 2, 1000);
	HAL_GPIO_WritePin(AD9833_CS_GPIO_Port, AD9833_CS_Pin, GPIO_PIN_SET);
}
void ad9833_set_wave_form(ad9833_waveform_t waveform){
	switch (waveform)

		{
		case SINUS:
			ad9833_write_16bit(0x2000);
			break;
		case SQUARE:
			ad9833_write_16bit(0x2028);
			break;
		case TRIANGLE:
			ad9833_write_16bit(0x2002);
			break;
		default:
			break;

		}
}
void ad9833_set_wave_param(ad9833_waveform_t waveform, float frequency,
		float phase)
{
	if (phase < 0)
		phase = 0; // Фаза тільки позитивна
	if (phase > 360)
		phase = 360;
	phaseVal = ((int) (phase * (4096 / 360))) | 0XC000; // 4096/360 = 11.37 change per Degree for Register And using 0xC000 which is Phase 0 Register Address
	// ---------- Tuning word for Frequency
	long freq = 0;
	freq = (int) (((frequency * pow(2, 28)) / FMCLK) + 1);
	FRQHW = (int) ((freq & 0xFFFC000) >> 14);
	FRQLW = (int) (freq & 0x3FFF);
	FRQLW |= 0x4000;
	FRQHW |= 0x4000;
	ad9833_write_16bit(0x2100);
	ad9833_write_16bit(FRQLW);
	ad9833_write_16bit(FRQHW);
	ad9833_write_16bit(phaseVal);
	ad9833_set_wave_form(waveform);
}

void ad9833_set_power(ad9833_power_t state)
{
	switch (state)
	{
	case POWER_OFF:
		ENABLE(CONTROL_REGISTER, STOP_DAC); // DAC power supply down
		ENABLE(CONTROL_REGISTER, STOP_MCLK); // stop AD9833 MCLK clock
		break;
	case POWER_ON:
		DISABLE(CONTROL_REGISTER, STOP_DAC); // DAC power supply up
		DISABLE(CONTROL_REGISTER, STOP_MCLK); // start AD9833 MCLK clock
		break;
	}
	ad9833_write_16bit(CONTROL_REGISTER); // AD9833 power up/down
}

