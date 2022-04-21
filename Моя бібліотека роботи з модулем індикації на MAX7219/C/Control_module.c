#include "Control_module.h"

uint8_t aTxBuf[1] =
{ 0 };
#define SPI_ITEM hspi1
extern SPI_HandleTypeDef SPI_ITEM;
char dg = 4;

/*  символи:  градус   t      F    E     r    s   */
char SEG[9] =
{ 0x63, 0x0F, 0x47, 0x4F, 0x05, 0x00, 0x6D };
/*                0    1      2     3     4     5     6    7     8      9 */
char NUM[10] =
{ 0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B };

#define cs_set() HAL_GPIO_WritePin(MAX7219_NCS_GPIO_Port, MAX7219_NCS_Pin, GPIO_PIN_RESET)
#define cs_reset() HAL_GPIO_WritePin(MAX7219_NCS_GPIO_Port, MAX7219_NCS_Pin, GPIO_PIN_SET)
extern TIM_HandleTypeDef htim2;
void MAX7219_write_register(uint8_t rg, uint8_t dt)
{
	cs_set();
	aTxBuf[0] = rg;
	HAL_SPI_Transmit(&SPI_ITEM, (uint8_t*) aTxBuf, 1, 5000);
	aTxBuf[0] = dt;
	HAL_SPI_Transmit(&SPI_ITEM, (uint8_t*) aTxBuf, 1, 5000);
	cs_reset();

}
//------------------------------------------------------
void MAX7219_clear(void)
{
	uint8_t i = dg;
	do
	{
		MAX7219_write_register(i, 0x00);
	} while (--i);
}

//-------------------------------------------------------
void MAX7219_init(void)
{

	MAX7219_write_register(0x09, 0x00);
	MAX7219_write_register(0x0B, dg - 1);
	MAX7219_set_brightness(13);
	MAX7219_write_register(0x0C, 0x01);
	MAX7219_clear();

}

void MAX7219_print_float(float number)
{

	float Buffer = number;
	int Left = Buffer;
	float Drob = Buffer - Left;

	//Виводжу цілу частину
	uint8_t DIG3 = Left % 100 / 10;
	uint8_t DIG4 = Left % 10;
	MAX7219_write_register(2, NUM[DIG3]);
	MAX7219_write_register(1, NUM[DIG4] | 0x80);

	//Вивід дробної частини (1 знак після коми)
	int Right = 0;

	Right = round(Drob * 100); // Округлюємо оскільки при діленні на число кратне 3 виникають небажані ефекти
	DIG3 = (int) Right % 100 / 10;
	DIG4 = (int) Right % 10;
	//MAX7219_write_register(1, NUM[DIG3]);
	MAX7219_write_register(4, NUM[DIG3]);

}

void MAX7219_set_brightness(uint8_t Intensity)
{
	if (Intensity > 15)
		Intensity = 15;
	if (Intensity < 0)
		Intensity = 0;
	MAX7219_write_register(0x0A, Intensity);

}

void MAX7219_print_char(uint8_t position, uint8_t symbol)
{
	{
		switch (position)
		{
		case 0:
			MAX7219_write_register(3, symbol);
			break;
		case 1:
			MAX7219_write_register(2, symbol);
			break;
		case 2:
			MAX7219_write_register(1, symbol);
			break;
		case 3:
			MAX7219_write_register(4, symbol);
			break;
		}

	}
}

void MAX7219_print_temperature(volatile uint16_t number)
{
	/*mode: 1 - у 0-й позиції виводиться символ t
	 *      2 - у 0-й позиції виводиться символ c*/
	volatile uint16_t n = 0;
	n = number;
	if (n >= 9999)
		n = 9999;
	if (n <= 0)
		n = 0;
	uint8_t DIG2 = n % 1000 / 100;
	uint8_t DIG3 = n % 100 / 10;
	uint8_t DIG4 = n % 10;

	//MAX7219_print_char(0, 0x0F); // 't'
	MAX7219_write_register(3, NUM[DIG2]);
	MAX7219_write_register(2, NUM[DIG3]);
	MAX7219_write_register(1, NUM[DIG4]);
	MAX7219_print_char(3, 0x63);

}

void MAX7219_print_int(volatile uint16_t number)
{
	volatile uint16_t n = 0;
	n = number;
	if (n >= 9999)
		n = 9999;
	if (n <= 0)
		n = 0;

	uint8_t DIG1 = n % 10000 / 1000;
	uint8_t DIG2 = n % 1000 / 100;
	uint8_t DIG3 = n % 100 / 10;
	uint8_t DIG4 = n % 10;

	MAX7219_write_register(3, NUM[DIG1]);
	MAX7219_write_register(2, NUM[DIG2]);
	MAX7219_write_register(1, NUM[DIG3]);
	MAX7219_write_register(4, NUM[DIG4]);

}

void LED1_set_state(bool state)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state);
}
void LED2_set_state(bool state)
{
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, state);
}
void Buzzer_beep(void)
{
	HAL_TIM_Base_Start_IT(&htim2);

}
