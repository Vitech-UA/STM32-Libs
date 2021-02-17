#include "RCC.h"
#include "stm32f051x8.h"

void MCO::SetChanel(MCO_CHANEL ch)
{
	// При вказаному PLL для STM32F05x MCO = PLLclk / 2
	RCC->CFGR &= ~RCC_CFGR_MCO;    // Clear bitfield
	RCC->CFGR |= (ch << RCC_CFGR_MCO_Pos);
}
void MCO::Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFRH0; // PA8 -> AF0 = MCO
	GPIOA->MODER &= ~GPIO_MODER_MODER8;     // Clear BitField
	GPIOA->MODER |= GPIO_MODER_MODER8_1;    // Set AF0
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8;     // Push-pull
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
}

Rcc::Rcc(HCLK_FREQ_t HclkFreq)
{

	this->freq = HclkFreq;
	// Init();
	Init(this->freq);
}
void Rcc::DisableHSE()
{
	RCC->CR &= ~(uint32_t) RCC_CR_HSEON;
}
void Rcc::EnableHSE()
{
	RCC->CR |= (uint32_t) RCC_CR_HSEON;                // Send command
	while (!(RCC->CR & RCC_CR_HSERDY))   // Wait HSE ready
	{
	}
}
void Rcc::EnableHSI()
{
	RCC->CR |= (uint32_t) RCC_CR_HSION;              // Send command
	while (!(RCC->CR & RCC_CR_HSIRDY)) // Wait HSI ready
	{
	}
}
void Rcc::DisableHSI()
{
	RCC->CR &= ~(uint32_t) RCC_CR_HSION;
}
void Rcc::EnablePLL()
{
	RCC->CR |= (uint32_t) RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY)) //Wait flag
	{

	}
}
void Rcc::DisablePLL()
{
	RCC->CR &= ~(uint32_t) RCC_CR_PLLON;
}
void Rcc::Set_SYSCLK_Source(SYSCLK_SOURCE_t SRC)
{
	RCC->CFGR &= ~(uint32_t) RCC_CFGR_SW;  // Clear SW bitfield
	RCC->CFGR |= (SRC << RCC_CFGR_SW_Pos);
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1)
	{
	}
}
void Rcc::Set_PLL_Multipler(PLL_MUL_t MUL)
{
	RCC->CR &= uint32_t(~RCC_CR_PLLON);
	while ((RCC->CR & RCC_CR_PLLRDY) != 0)
	{
	} // wait PLL unlocking
	RCC->CFGR &= ~(uint32_t) RCC_CFGR_PLLMUL;   // Clear PLLMUL bitfield

	RCC->CFGR |= (MUL << RCC_CFGR_PLLMUL_Pos);
	//RCC->CFGR |= (uint32_t)RCC_CFGR_PLLMUL2;
}
void Rcc::Set_PLL_Source(PLL_SRC_t SRC)
{
	RCC->CFGR &= ~RCC_CFGR_PLLSRC; // Clear PLLSRC bitfield
	RCC->CFGR |= (SRC << RCC_CFGR_PLLSRC_Pos); //set pll source
}
void Rcc::ConfigFlashMemoryClk(void)
{

	FLASH->ACR |= FLASH_ACR_LATENCY; // 001
	/*
	 000: Zero wait state, if SYSCLK ≤ 24 MHz
	 001: One wait state, if 24 MHz < SYSCLK ≤ 48 MHz
	 */
	FLASH->ACR |= FLASH_ACR_PRFTBE;   // Prefetch buffer enable
}
void Rcc::Set_AHB_Prescaler(AHB_PRSC_t psc)
{
	RCC->CFGR &= ~RCC_CFGR_HPRE;        // Clear  AHB (HCLK) prescaler bit field
	RCC->CFGR |= (psc << RCC_CFGR_HPRE_Pos); // Set Divider psc
}
void Rcc::Set_APB_Prescaler(APB_PRSC_t psc)
{
	RCC->CFGR &= ~RCC_CFGR_PPRE; // // Clear  APB (PCLK) prescaler bit field
	RCC->CFGR |= (psc << RCC_CFGR_PPRE_Pos);
}
void Rcc::Set_PREDIV(PREDIV_PRSC_t psc)
{
	RCC->CFGR2 &= ~RCC_CFGR2_PREDIV; // Clear Prediv BitField
	RCC->CFGR2 |= (psc << RCC_CFGR2_PREDIV_Pos);
}

void Rcc::Init(HCLK_FREQ_t HclkFreq)
{

#if(STM32_SERIES == 0)
	// Init RCC module
	this->EnableHSI();                // HSI = 8MHz, але на PLL зайде лише 4 Мгц
	this->ConfigFlashMemoryClk();
	this->Set_AHB_Prescaler(AHB_NOT_DIVIDED);
	this->Set_APB_Prescaler(APB_NOT_DIVIDED);

	switch (HclkFreq)
	{
	case HCLK_1MHz:
		this->Set_PLL_Multipler(PLL_MULx2);
		this->Set_AHB_Prescaler(AHB_DIV_BY_8);
		break;
	case HCLK_2MHz:
		this->Set_PLL_Multipler(PLL_MULx2);
		this->Set_AHB_Prescaler(AHB_DIV_BY_4);
		break;
	case HCLK_4MHz:
		this->Set_PLL_Multipler(PLL_MULx2);
		this->Set_AHB_Prescaler(AHB_DIV_BY_2);
		break;
	case HCLK_8MHz:
		this->Set_PLL_Multipler(PLL_MULx2);
		break;
	case HCLK_16MHz:
		this->Set_PLL_Multipler(PLL_MULx4);
		break;
	case HCLK_32MHz:
		this->Set_PLL_Multipler(PLL_MULx8);
		break;
	case HCLK_48MHz:
		this->Set_PLL_Multipler(PLL_MULx12);
		break;
	default:
		this->Set_PLL_Multipler(PLL_MULx2);
		break;
	}
	this->Set_PLL_Source(HSI_DIV_By2);      // PLL in 4 Мhz
	this->EnablePLL();
	this->Set_SYSCLK_Source(PLL);
#endif
}

