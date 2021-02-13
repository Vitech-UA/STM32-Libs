/*
 * ControlModule.cpp
 *
 *  Created on: Jan 23, 2021
 *      Author: Embed Viktor
 */
#include <ControlModule.h>
#include <math.h>
//            'град'   't'    'F'  'E'    'r'  ' '   'S'
char CHAR[9] = { 0x63, 0x0F, 0x47, 0x4F, 0x05, 0x00, 0x6D };
/*                0    1      2     3     4     5     6    7     8      9 */
char NUM[10] = { 0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B };

/*************************MAX7219*********************************/
ControlModule::ControlModule() {
// default constructor

}
void ControlModule::Init() {

	this->ConfigGPIO(GPIOA, 5, GPIOB, 5, GPIOA, 4);

	this->nCS_HIGH();

	this->ConfigMAX7219(SPI1);

	this->Transmit(0x09, 0x00);
	this->Transmit(0x0B, this->dg - 1);
	this->Transmit(0x0A, 0x02);
	this->Transmit(0x0C, 0x01);
	this->Clear();
}
void Send2Byte(uint16_t data) {

}
void ControlModule::ConfigMAX7219(SPI_TypeDef *SPI_PORT) {

	this->SPI_ITEM = SPI_PORT;

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	this->SPI_ITEM->CR1 |= SPI_CR1_BIDIMODE; // 1: line bidirectional data mode selected
	this->SPI_ITEM->CR1 |= SPI_CR1_BIDIOE; // 1: Output enabled (transmit-only mode)
	this->SPI_ITEM->CR1 |= SPI_CR1_MSTR;     // 1: Master configuration
	this->SPI_ITEM->CR1 &= ~SPI_CR1_BR;      // 000: fPCLK/2
	this->SPI_ITEM->CR1 &= ~SPI_CR1_LSBFIRST; // 0: data is transmitted / received with the MSB first
	this->SPI_ITEM->CR1 |= SPI_CR1_SSM;  // 1: Software slave management enabled
	this->SPI_ITEM->CR1 |= SPI_CR1_SSI;
	this->SPI_ITEM->CR2 &= ~SPI_CR2_DS;      //Clear bitfield
	this->SPI_ITEM->CR2 |= 0x0F << SPI_CR2_DS_Pos; // 16 Bit frame
	this->SPI_ITEM->CR1 &= ~SPI_CR1_CPOL;    // Polarity cls signal CPOL = 0;
	this->SPI_ITEM->CR1 &= ~SPI_CR1_CPHA;    // Phase cls signal    CPHA = 0;
	this->SPI_ITEM->CR2 &= ~SPI_CR2_FRF;     // Motorolla mode
	this->SPI_ITEM->CR1 |= SPI_CR1_SPE;

}
void ControlModule::ConfigGPIO(GPIO_TypeDef *clk_port, uint16_t clk_pin,
		GPIO_TypeDef *din_port, uint16_t din_pin, GPIO_TypeDef *load_port,
		uint16_t load_pin) {
	/*
	 * PA5 -> SPI1-SCK(AF0)
	 * PB5 -> SPI1-MOSI(AF0)
	 */
	this->CLK_PORT = clk_port;
	this->CLK_PIN = clk_pin;
	this->DIN_PORT = din_port;
	this->DIN_PIN = din_pin;
	this->LOAD_PORT = load_port;
	this->LOAD_PIN = load_pin;

	GpioEnableClk(this->CLK_PORT);
	GpioEnableClk(this->DIN_PORT);
	GpioEnableClk(this->LOAD_PORT);

	/*Настройка clk-pin (HI SPEED, AF, PUSH-PULL)*/
	GpioSetAsAF(this->CLK_PORT, this->CLK_PIN, AF0);
	this->CLK_PORT->OTYPER &= ~(1 << this->CLK_PIN); // Push-pull
	this->CLK_PORT->OSPEEDR |= 0x03 << (this->CLK_PIN * 2);

	/*Настройка mosi-pin (HI SPEED, AF, PUSH-PULL)*/
	GpioSetAsAF(this->DIN_PORT, this->DIN_PIN, AF0);
	this->DIN_PORT->OTYPER &= ~(1 << this->DIN_PIN); // Push-pull
	this->DIN_PORT->OSPEEDR |= 0x03 << (this->DIN_PIN * 2);

	/*Настройка load-pin (HI SPEED, GP_Output, PUSH-PULL)*/
	this->LOAD_PORT->MODER |= 0x01 << (this->LOAD_PIN * 2); // GP Output
	this->LOAD_PORT->OTYPER &= ~(1 << this->LOAD_PIN); // Push-pull
	this->LOAD_PORT->OSPEEDR |= 0x03 << (this->LOAD_PIN * 2); // Hi Speed
	this->nCS_HIGH();
}
void ControlModule::nCS_LOW(void) {
	this->LOAD_PORT->BSRR |= (1 << this->LOAD_PIN) << 16U;
}
void ControlModule::nCS_HIGH(void) {
	this->LOAD_PORT->BSRR |= (1 << this->LOAD_PIN);
}
void ControlModule::Transmit(uint8_t rg, uint8_t dt) {

	this->nCS_LOW();
	this->SPI_ITEM->DR = (uint16_t) ((rg << 8) | dt);
	while ((this->SPI_ITEM->SR & SPI_SR_BSY) == SPI_SR_BSY)
		;
	this->nCS_HIGH();
}
void ControlModule::Clear(void) {
	uint8_t i = this->dg;
	do {
		this->Transmit(i, 0x00);
	} while (--i);

}
void ControlModule::PrintInt(volatile uint16_t number) {
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

	this->Transmit(3, NUM[DIG1]);
	this->Transmit(2, NUM[DIG2]);
	this->Transmit(1, NUM[DIG3]);
	this->Transmit(4, NUM[DIG4]);

}

void ControlModule::PrintFloat(float number) {

	float Buffer = number;
	int Left = Buffer;
	float Drob = Buffer - Left;

	//Виводжу цілу частину
	uint8_t DIG3 = Left % 100 / 10;
	uint8_t DIG4 = Left % 10;
	this->Transmit(3, NUM[DIG3]);
	this->Transmit(2, NUM[DIG4] | 0x80);

	//Вивід дробної частини
	int Right = 0;

	Right = round(Drob * 100); // Округлюємо оскільки при діленні на число кратне 3 виникають небажані ефекти
	DIG3 = (int) Right % 100 / 10;
	DIG4 = (int) Right % 10;
	this->Transmit(1, NUM[DIG3]);
	this->Transmit(4, NUM[DIG4]);

}

void ControlModule::PrintChar(uint8_t rg, uint8_t dt) {
	switch (rg) {
	case 0:
		this->Transmit(3, dt);
		break;
	case 1:
		this->Transmit(2, dt);
		break;
	case 2:
		this->Transmit(1, dt);
		break;
	case 3:
		this->Transmit(4, dt);
		break;
	}

}

void ControlModule::SetBrightness(uint8_t Intensity) {
	if (Intensity > 15)
		Intensity = 15;
	if (Intensity < 0)
		Intensity = 0;
	this->Transmit(0x0A, Intensity);

}
/*************************LED************************************/
void Led::SetState(LED_STATE_t LedState) {
	switch (LedState) {
	case LED_OFF:
		this->Port->BSRR |= (1 << this->Pin) << 16U; // Reset
		break;
	case LED_ON:
		this->Port->BSRR |= (1 << this->Pin); // Set
		break;
	}
}
void Led::Config(GPIO_TypeDef *Led_Port, uint16_t Led_Pin) {

#if (STM32Fx == 0)
	/*Присвоюю йому пін & порт*/
	this->Port = Led_Port;
	this->Pin = Led_Pin;
	/*Ввімкнути тактування обраного GPIO*/
	GpioEnableClk(Led_Port);
	/*Встановити як вихід пуш-пул*/
	this->Port->MODER |= (0x01 << (Pin * 2)); // GPIO->To output
	this->Port->PUPDR |= (0x01 << (Pin * 2)); // PullUp(Св.діод керується катодом)
	this->Port->BSRR |= (1 << this->Pin) << 16U;
#endif
}

/*************************BUTTON************************************/
void Button::Config(GPIO_TypeDef *ButtonPort, uint16_t ButtonPin) {
	this->Port = ButtonPort;
	this->Pin = ButtonPin;
	GpioEnableClk(ButtonPort);
}
BUTTON_STATE_t Button::GetState(void) {

	if (this->Port->IDR & (0x01 << this->Pin))
		return RELEASED;
	else
		return PRESSED;

}

/*************************BUZZER************************************/
void Buzzer::Config(GPIO_TypeDef *BuzzerPort, uint16_t BuzzerPin) {
	this->Port = BuzzerPort;
	this->Pin = BuzzerPin;
	GpioEnableClk(BuzzerPort);

	/*Встановити як вихід пуш-пул*/
	this->Port->MODER |= (0x01 << (this->Pin * 2)); // GPIO->To output
	this->Port->PUPDR |= (0x01 << (this->Pin * 2)); // PullUp(Св.діод керується катодом)
	this->Port->BSRR |= (1 << this->Pin) << 16U;

}
void Buzzer::SetState(BUZZER_STATE_t BuzzerState) {
	switch (BuzzerState) {
	case ON:
		this->Port->BSRR |= (1 << this->Pin); // Reset
		break;
	case OFF:
		this->Port->BSRR |= (1 << this->Pin) << 16U; // Reset
		break;
	}
}

/*************************COMMON************************************/
void GpioEnableClk(GPIO_TypeDef *PORT) {
#if (STM32Fx == 0)
	if (PORT == GPIOA) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOAEN))
			RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	} else if (PORT == GPIOB) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOBEN))
			RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	} else if (PORT == GPIOC) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOCEN))
			RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	} else if (PORT == GPIOD) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOCEN))
			RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	} else if (PORT == GPIOF) {
		if (!(RCC->AHBENR & RCC_AHBENR_GPIOFEN))
			RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
	}

#endif
}
void GpioSetAsAF(GPIO_TypeDef *PORT, uint16_t gpio_pin, AF_t AF) {

	if (gpio_pin <= 7) // For gpio.0 ... gpio.7
			{
		PORT->MODER |= 0x02 << (gpio_pin * 2);
		PORT->AFR[0] |= AF << (gpio_pin * 4);
	} else if (gpio_pin > 7 && gpio_pin <= 15) // For gpio.8 ... gpio.15
			{
		PORT->MODER |= 0x02 << (gpio_pin * 2);
		PORT->AFR[1] |= AF << ((gpio_pin - 8) * 4);
	} else // For gpio.unkown
	{
		while (1) {
			// Infinite error loop :-(
		}
	}

}
