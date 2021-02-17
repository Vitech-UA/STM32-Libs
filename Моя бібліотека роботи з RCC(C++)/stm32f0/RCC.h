#ifndef RCC_H_
#define RCC_H_

#define STM32_SERIES 0

/*

 INFO:
 Для ініціалізації модуль RCC, достатньо створити об'єкт його класу у функції main.
 Ініціалізація відбувається автоматично в конструкторі.

 */

#include "stm32f051x8.h"

#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *) __FILE__, __LINE__))

#define IS_MCO_TYPE(TYPE) (((TYPE) == MCO_DISABLED) || ((TYPE) == MCO_HSI_14Mhz) || ((TYPE) == MCO_LSI) || \
 ((TYPE) == MCO_LSE) || ((TYPE) == MCO_SystemClock)||((TYPE) == MCO_HSI_8Mhz)||((TYPE) == MCO_HSE))||((TYPE) == MCO_PLL))||((TYPE) == MCO_HSI48))

#if (STM32_SERIES == 0)
typedef enum HCLK_FREQ {
	HCLK_1MHz = 0,
	HCLK_2MHz,
	HCLK_4MHz,
	HCLK_8MHz,
	HCLK_16MHz,
	HCLK_32MHz,
	HCLK_48MHz

} HCLK_FREQ_t;
#endif

typedef enum MCO_CHANEL {

	//Актуально для STM32F051R8T6
	MCO_DISABLED = 0,
	MCO_HSI_14Mhz,
	MCO_LSI,
	MCO_LSE,
	MCO_SystemClock,
	MCO_HSI_8Mhz,
	MCO_HSE,
	MCO_PLL,
	MCO_HSI48

/*
 0000: MCO output disabled, no clock on MCO
 0001: Internal RC 14 MHz (HSI14) oscillator clock selected
 0010: Internal low speed (LSI) oscillator clock selected
 0011: External low speed (LSE) oscillator clock selected
 0100: System clock selected
 0101: Internal RC 8 MHz (HSI) oscillator clock selected
 0110: External 4-32 MHz (HSE) oscillator clock selected
 0111: PLL clock selected (divided by 1 or 2, depending on PLLNODIV)
 1000: Internal RC 48 MHz (HSI48) oscillator clock selected
 */

} MCO_CHANEL;
typedef enum HCLK_PRESCELAR {
	HCLK_NOT_DIVIDED,
	HCLK_DIV_BY_2,
	HCLK_DIV_BY_4,
	HCLK_DIV_BY_8,
	HCLK_DIV_BY_16

} APB1_PRESCALER_t, APB2_PRESCALER_t;
typedef enum SYSCLK_SOURCE {
	HSI, HSE, PLL, HSI48

} SYSCLK_SOURCE_t;
typedef enum SYSCLK_PRESCELAR {
	SYSCLK_NOT_DIVIDED,
	SYSCLK_DIV_BY_2,
	SYSCLK_DIV_BY_4,
	SYSCLK_DIV_BY_8,
	SYSCLK_DIV_BY_16,
	SYSCLK_DIV_BY_64,
	SYSCLK_DIV_BY_128,
	SYSCLK_DIV_BY_256,
	SYSCLK_DIV_BY_512,

} AHB_PRESCALER_t;
typedef enum PLL_MULTIPLER {
	PLL_MULx2 = 0,
	PLL_MULx3,
	PLL_MULx4,
	PLL_MULx5,
	PLL_MULx6,
	PLL_MULx7,
	PLL_MULx8,
	PLL_MULx9,
	PLL_MULx10,
	PLL_MULx11,
	PLL_MULx12,
	PLL_MULx13,
	PLL_MULx14,
	PLL_MULx15,
	PLL_MULx16,
} PLL_MUL_t;
typedef enum PLL_SRC {
	HSI_DIV_By2 = 0, HSI_PREDIV, HSE_PREDIV, HSI48_PREDIV
} PLL_SRC_t;
typedef enum APB_PRCS {
	APB_NOT_DIVIDED = 0, APB_DIV_BY_2, APB_DIV_BY_4, APB_DIV_BY_8, APB_DIV_BY_16
} APB_PRSC_t;
typedef enum AHB_PRCS {
	AHB_NOT_DIVIDED = 0,
	AHB_DIV_BY_2,
	AHB_DIV_BY_4,
	AHB_DIV_BY_8,
	AHB_DIV_BY_16,
	AHB_DIV_BY_32,
	AHB_DIV_BY_64,
	AHB_DIV_BY_128,
	AHB_DIV_BY_256,
	AHB_DIV_BY_512,
} AHB_PRSC_t;
typedef enum PREDIV_PRSC {
	PREDIV_INPUT_CLOCK_NOT_DIV = 0, // 0000
	PREDIV_INPUT_CLOCK_DIV_BY_2,
	PREDIV_INPUT_CLOCK_DIV_BY_3,
	PREDIV_INPUT_CLOCK_DIV_BY_4,
	PREDIV_INPUT_CLOCK_DIV_BY_5,
	PREDIV_INPUT_CLOCK_DIV_BY_6,
	PREDIV_INPUT_CLOCK_DIV_BY_7,
	PREDIV_INPUT_CLOCK_DIV_BY_8,
	PREDIV_INPUT_CLOCK_DIV_BY_9,
	PREDIV_INPUT_CLOCK_DIV_BY_10,
	PREDIV_INPUT_CLOCK_DIV_BY_11,
	PREDIV_INPUT_CLOCK_DIV_BY_12,
	PREDIV_INPUT_CLOCK_DIV_BY_13,
	PREDIV_INPUT_CLOCK_DIV_BY_14,
	PREDIV_INPUT_CLOCK_DIV_BY_15,
	PREDIV_INPUT_CLOCK_DIV_BY_16    // 1111

} PREDIV_PRSC_t;
class MCO {
public:
	void SetChanel(MCO_CHANEL ch);
	void Init(void);
};

class Rcc {
public:
	Rcc(HCLK_FREQ_t HclkFreq);
	void Init(HCLK_FREQ_t HclkFreq);
private:

	static void EnableHSE(void);
	static void DisableHSE(void);
	static void EnablePLL(void);
	static void DisablePLL(void);
	static void EnableCSS(void);
	static void DisableCSS(void);
	static void EnableHSI(void);
	static void DisableHSI(void);
	static void Set_SYSCLK_Source(SYSCLK_SOURCE_t SRC);
	static void Set_PLL_Multipler(PLL_MUL_t MUL);
	static void Set_PLL_Source(PLL_SRC_t SRC);
	static void ConfigFlashMemoryClk(void);
	static void Set_AHB_Prescaler(AHB_PRSC_t psc);
	static void Set_APB_Prescaler(APB_PRSC_t psc);
	static void Set_PREDIV(PREDIV_PRSC_t psc);
	HCLK_FREQ_t freq;
};
#endif /* RCC_H_ */
