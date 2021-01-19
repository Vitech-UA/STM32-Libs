#include "RCC.h"

#pragma region MCO
void MCO::SetChanel(MCO_CHANEL ch)
{
	// При вказаному PLL для STM32F05x MCO = PLLclk / 2
	RCC->CFGR &= ~RCC_CFGR_MCO;    // Clear bitfield
    RCC->CFGR |= (ch << RCC_CFGR_MCO_Pos);	
}
void MCO::Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	//GPIOA->AFR[1] &= ~GPIO_AFRH_AFRH0; // PA8 -> AF0 = MCO
	GPIOA->MODER &= ~GPIO_MODER_MODER8;     // Clear BitField
	GPIOA->MODER |= GPIO_MODER_MODER8_1;    // Set AF0
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8;     // Push-pull
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8; 
}	
#pragma endregion

#pragma region RCC
void Clock::DisableHSE()
{
	RCC->CR &= ~(uint32_t)RCC_CR_HSEON;
}
void Clock::EnableHSE()
{ 
	RCC->CR |= (uint32_t)RCC_CR_HSEON;                // Send command
	while(!(RCC->CR & RCC_CR_HSERDY))   // Wait HSE ready
	{}
}
void Clock::EnableHSI()
{
	RCC->CR |= (uint32_t)RCC_CR_HSION;              // Send command
	while(!(RCC->CR & RCC_CR_HSIRDY)) // Wait HSI ready
	{}
}
void Clock::DisableHSI()
{
	RCC->CR &= ~(uint32_t)RCC_CR_HSION;
}
void Clock::EnablePLL()
{
	RCC->CR |= (uint32_t)RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY)) //Wait flag
		{
		
		}
	
}
void Clock::DisablePLL()
{
	RCC->CR &= ~(uint32_t)RCC_CR_PLLON;
}
void Clock::Set_SYSCLK_Source(SYSCLK_SOURCE_t SRC)
{
	RCC->CFGR &= ~(uint32_t)RCC_CFGR_SW;  // Clear SW bitfield
	RCC->CFGR |= (SRC << RCC_CFGR_SW_Pos);
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1)
	{
		
	}
	
}
void Clock::Set_PLL_Multipler(PLL_MUL_t MUL)
{
	RCC->CR &= uint32_t(~RCC_CR_PLLON_Msk);
	//while ((RCC->CR & RCC_CR_PLLRDY) != 0) {} // wait PLL unlocking
		RCC->CFGR &= ~(uint32_t)RCC_CFGR_PLLMUL;   // Clear PLLMUL bitfield
	
	//RCC->CFGR |= (MUL << RCC_CFGR_PLLMUL_Pos);
	RCC->CFGR |= (uint32_t)RCC_CFGR_PLLMUL2;
}
void Clock::Set_PLL_Source(PLL_SRC_t SRC)
{
	RCC->CFGR &= ~RCC_CFGR_PLLSRC; // Clear PLLSRC bitfield
	RCC->CFGR |= (SRC << RCC_CFGR_PLLSRC_Pos); //set pll source
}
void Clock::ConfigFlashMemoryClk(void)
{
	FLASH->ACR |= FLASH_ACR_LATENCY; // 001
	/*
	 000: Zero wait state, if SYSCLK ≤ 24 MHz
     001: One wait state, if 24 MHz < SYSCLK ≤ 48 MHz
    */
	FLASH->ACR |= FLASH_ACR_PRFTBE;   // Prefetch buffer enable

}
void Clock::Set_AHB_Prescaler(AHB_PRSC_t psc) 
{
	RCC->CFGR &= ~RCC_CFGR_HPRE;             // Clear  AHB (HCLK) prescaler bit field
	RCC->CFGR |= (psc << RCC_CFGR_HPRE_Pos); // Set Divider psc
}
void Clock::Set_APB_Prescaler(APB_PRSC_t psc)
{
	RCC->CFGR &= ~RCC_CFGR_PPRE; // // Clear  APB (PCLK) prescaler bit field
	RCC->CFGR |= (psc << RCC_CFGR_PPRE_Pos);
}
void Clock::Set_PREDIV(PREDIV_PRSC_t psc)
{
	RCC->CFGR2 &= ~RCC_CFGR2_PREDIV; // Clear Prediv BitField
	RCC->CFGR2 |= (psc << RCC_CFGR2_PREDIV_Pos);
}

void Clock::Init(void)
{
	// Init RCC module
	
	this->EnableHSI();                     // HSI = 8MHz, але на PLL зайде лише 4 Мгц
	this->ConfigFlashMemoryClk();
	this->Set_AHB_Prescaler(AHB_NOT_DIVIDED);
	this->Set_APB_Prescaler(APB_NOT_DIVIDED);
	this->Set_PLL_Source(HSI_PREDIV);      // PLL in 4 Мhz
	this->Set_PREDIV(PREDIV_INPUT_CLOCK_DIV_BY_4);
	this->Set_PLL_Multipler(PLL_MULx10);
	this->EnablePLL();
	this->Set_SYSCLK_Source(PLL);
	
		
	
	
}
#pragma endregion
