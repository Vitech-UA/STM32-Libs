#include "RCC.h"

#pragma region MCO


void MCO::SetChanel(MCO_CHANEL ch)
{
	RCC->CFGR &= ~RCC_CFGR_MCO;   // Clear bitfield
    RCC->CFGR |= (ch << RCC_CFGR_MCO_Pos);	
}

void MCO::Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	//GPIOA->AFR[1] &= ~GPIO_AFRH_AFRH0; // PA8 -> AF0 = MCO
	GPIOA->MODER &= ~GPIO_MODER_MODER8;    // Clear BitField
	GPIOA->MODER |= GPIO_MODER_MODER8_1;   // Set AF0
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8;    // Push-pull
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8; 
}	

#pragma endregion

#pragma region RCC

void Clock::DisableHSE()
{
	RCC->CR &= ~RCC_CR_HSEON;
}

void Clock::EnableHSE()
{ 
	RCC->CR |= RCC_CR_HSEON;              // Send command
	while(!(RCC->CR & RCC_CR_HSERDY))   // Wait HSE ready
	{}
}

void Clock::EnableHSI()
{
	RCC->CR |= RCC_CR_HSION;            // Send command
	while(!(RCC->CR & RCC_CR_HSIRDY)) // Wait HSI ready
	{}
}

void Clock::DisableHSI()
{
	RCC->CR &= ~RCC_CR_HSION;
}

void Clock::EnablePLL()
{
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY)) //Wait flag
		{
		
		}
	
}

void Clock::DisablePLL()
{
	RCC->CR &= ~RCC_CR_PLLON;
}

void Clock::Set_SYSCLK_Source(SYSCLK_SOURCE_t SRC)
{
	RCC->CFGR |= (SRC << RCC_CFGR_SW_Pos);
}

void Clock::Set_PLL_Multipler(PLL_MUL_t MUL)
{
	RCC->CFGR |= (MUL << RCC_CFGR_PLLMUL_Pos);
	
}

void Clock::Set_PLL_Source(PLL_SRC_t SRC)
{
	
}

void Clock::Init(void)
{
	// Init RCC module
	this->EnableHSI(); // HSI = 8MHz, але на PLL зайде лише 4 
	this->EnablePLL();
	this->Set_PLL_Source(HSI_PREDIV);
	this->Set_SYSCLK_Source(PLL);	
	this->Set_PLL_Multipler(PLL_MULx12); // Multiple x6: 8MHz * 6 = 48MHz(Maximum for STM32F051R8T6)
}
#pragma endregion
