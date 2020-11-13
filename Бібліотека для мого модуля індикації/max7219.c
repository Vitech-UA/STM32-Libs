#include "max7219.h"

uint8_t aTxBuf[1]={0};
extern SPI_HandleTypeDef SPI_ITEM;
char dg=4;

/*             град   t      F    E     r    */
char SEG[9] = {0x63, 0x0F, 0x47, 0x4F, 0x05, 0x00};
/*                0    1      2     3     4     5     6    7     8      9 */
char NUM[10] = {0x7E, 0x30, 0x6D ,0x79 ,0x33 ,0x5B ,0x5F ,0x70 ,0x7F , 0x7B};

#define cs_set() HAL_GPIO_WritePin(LOAD_PORT, LOAD_PIN, GPIO_PIN_RESET)
#define cs_reset() HAL_GPIO_WritePin(LOAD_PORT, LOAD_PIN, GPIO_PIN_SET)

void Send_7219 (uint8_t rg, uint8_t dt)
{
	cs_set();
	aTxBuf[0]=rg;
	HAL_SPI_Transmit (&SPI_ITEM, (uint8_t*)aTxBuf, 1, 5000);
	aTxBuf[0]=dt;
	HAL_SPI_Transmit (&SPI_ITEM, (uint8_t*)aTxBuf, 1, 5000);
	cs_reset();

}
//------------------------------------------------------
void Clear_7219 (void)
{
	uint8_t i=dg;
	do
	{
		Send_7219(i,0x00);//символ пустоты
	} while (--i);
}

//-------------------------------------------------------
void Init_7219 (void)
{

	    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	    RCC->AHBENR |=RCC_AHBENR_GPIOAEN;


	    SPI1->CR1  //Размер кадра 8 бит
	        | 0<<SPI_CR1_LSBFIRST_Pos     //MSB first
	        | 1<<SPI_CR1_SSM_Pos          //Программное управление SS
	        | 1<<SPI_CR1_SSI_Pos          //SS в высоком состоянии
	        | 0x04<<SPI_CR1_BR_Pos        //Скорость передачи: F_PCLK/32
	        | 1<<SPI_CR1_MSTR_Pos         //Режим Master (ведущий)
	        | 0<<SPI_CR1_CPOL_Pos | 0<<SPI_CR1_CPHA_Pos; //Режим работы SPI: 0

	      SPI1->CR1 |= 1<<SPI_CR1_SPE_Pos; //Включаем SPI



		Send_7219(0x09,0x00); // виключим режим декодирования
		Send_7219(0x0B,dg-1); // кол-во используемых разрядов
		Send_7219(0x0A,0x02); // интенсивность свечения
		Send_7219(0x0C,0x01); // включим индикатор
		Clear_7219();

}

void MAX7219_Print_Number(volatile uint16_t number)
{
	   volatile  uint16_t n=0;
	   n = number;
	   if(n >= 9999) n = 9999;
	   if(n <= 0) n = 0;

	   uint8_t DIG1 = n % 10000 / 1000;
	   uint8_t DIG2 = n % 1000 / 100;
	   uint8_t DIG3 = n % 100 / 10;
	   uint8_t DIG4 = n % 10;

	   Send_7219(3,NUM[DIG1]);
	   Send_7219(2,NUM[DIG2]);
	   Send_7219(1,NUM[DIG3]);
	   Send_7219(4,NUM[DIG4]);

}
