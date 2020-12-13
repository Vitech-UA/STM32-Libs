/*
 * timer.c
 *
 *  Created on: Dec 12, 2020
 *      Author: Embed Viktor
 */

#include "timer.h"

uint32_t aSRC_Buffer[BUFFER_DATA_NUMBER] = { 800 - 1, 1, 800, 800 - 1, 0, 8500,
		4000 - 1, 2, 2000 };
//uint32_t aSRC_Buffer[BUFFER_DATA_NUMBER] = { 1000, 1, 2000, 1, 3000,1, 4000 ,1, 5000 };

uint16_t Tim1Prescaler = 0;

void DelayUs(uint16_t Delay_Us) {

	uint16_t tim_psc_val = 0;
	tim_psc_val = SYSCLK / 1000000 - 1;
	TIMER_FOR_DELAY->PSC = tim_psc_val;
	TIMER_FOR_DELAY->SR = 0;             // Очистка флага Update event
	TIMER_FOR_DELAY->ARR = Delay_Us;         // Сюди записуємо бажану затримку
	TIMER_FOR_DELAY->CR1 |= TIM_CR1_CEN; // Вмикаємо таймер
	while (!(TIMER_FOR_DELAY->SR & TIM_SR_UIF))
		;
}
void DelayMs(uint16_t Delay_Ms) {
	uint16_t tim_psc_val = 0;
	tim_psc_val = SYSCLK / 1000 - 1;
	TIMER_FOR_DELAY->PSC = tim_psc_val;
	TIMER_FOR_DELAY->SR = 0;                 // Очистка флага Update event
	TIMER_FOR_DELAY->ARR = Delay_Ms;         // Сюди записуємо бажану затримку
	TIMER_FOR_DELAY->CR1 |= TIM_CR1_CEN;     // Вмикаємо таймер
	while (!(TIMER_FOR_DELAY->SR & TIM_SR_UIF))
		;
}
void DelayInit(void) {
#ifdef TIM6
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

#endif

#ifdef TIM7
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

#endif

}
void PWM_Init(void) {

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Ініціалізація PA8, як канал 1 Таймера 1.
	GPIOA->MODER |= GPIO_MODER_MODER8_1;
	/*
	 00: Input mode (reset state)
	 01: General purpose output mode
	 10: Alternate function mode !!!
	 11: Analog mode
	 */
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
	/*
	 x0: Low speed
	 01: Medium speed
	 11: High speed !!!
	 */
	GPIOA->AFR[1] |= 0x00000006; /* AF6: TIM1_CH1*/

	// Ініціалізація таймера
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // En clk

	TIMER_FOR_PWM->PSC = 3; /*For PWM_FREQ = 2 kHz */

	TIMER_FOR_PWM->ARR = 1000; // мах pwm value

	// PWM_FREQ = SYSCLK / ((PSC+1) * (ARR + 1))

	TIMER_FOR_PWM->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2
			| TIM_CCMR1_OC1M_3;

	/*
	 0110: PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
	 else inactive. In downcounting, channel 1 is inactive (OC1REF=‘0’) as long as
	 TIMx_CNT>TIMx_CCR1 else active (OC1REF=’1’).
	 0111: PWM mode 2 - In upcounting, channel 1 is inactive as long as
	 TIMx_CNT<TIMx_CCR1 else active. In downcounting, channel 1 is active as long as
	 TIMx_CNT>TIMx_CCR1 else inactive.
	 */
	//TIMER_FOR_PWM->CCR1 = TIM1->ARR / 2;
	TIMER_FOR_PWM->CCER |= TIM_CCER_CC1E;
	/*
	 0: Capture mode disabled / OC1 is not active (see below)
	 1: Capture mode enabled / OC1 signal is output on the corresponding output pin
	 *
	 */

	TIMER_FOR_PWM->BDTR |= TIM_BDTR_MOE;
	/*
	 MOE: Main output enable
	 This bit is cleared asynchronously by hardware as soon as one of the break inputs is active
	 (BRK or BRK2). It is set by software or automatically depending on the AOE bit. It is acting
	 only on the channels which are configured in output.
	 0: In response to a break 2 event. OC and OCN outputs are disabled
	 In response to a break event or if MOE is written to 0: OC and OCN outputs are disabled
	 or forced to idle state depending on the OSSI bit.
	 1: OC and OCN outputs are enabled if their respective enable bits are set (CCxE, CCxNE in
	 TIMx_CCER register).
	 */
	TIMER_FOR_PWM->CR1 |= TIM_CR1_CEN;
	/*
	 CEN: Counter enable
	 0: Counter disabled
	 1: Counter enabled

	 */

}
void PWM_SetDutyCycle(uint16_t Duty) {
	uint16_t ARR = TIMER_FOR_PWM->ARR;
	if (Duty > ARR)
		Duty = ARR;

	TIMER_FOR_PWM->CCR1 = Duty;

}
void OnePwmPulseModeInit(uint16_t PulseLengthUs, uint16_t PulseCount) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// Ініціалізація PA8, як канал 1 Таймера 1.
	GPIOA->MODER |= GPIO_MODER_MODER8_1;
	/*
	 00: Input mode (reset state)
	 01: General purpose output mode
	 10: Alternate function mode !!!
	 11: Analog mode
	 */
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
	/*
	 x0: Low speed
	 01: Medium speed
	 11: High speed !!!
	 */
	GPIOA->AFR[1] |= 0x00000006; /* AF6: TIM1_CH1*/
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // Дозвіл тактування TIM1
	uint16_t Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
	TIM1->CR1 &= ~TIM_CR1_CMS;
	/*
	 CMS[1:0]: Center-aligned mode selection
	 00: Edge-aligned mode. The counter counts up or down depending on the direction bit
	 (DIR).
	 01: Center-aligned mode 1. The counter counts up and down alternatively. Output compare
	 interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are
	 set only when the counter is counting down.
	 10: Center-aligned mode 2. The counter counts up and down alternatively. Output compare
	 interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are
	 set only when the counter is counting up.
	 11: Center-aligned mode 3. The counter counts up and down alternatively. Output compare
	 interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are
	 set both when the counter is counting up or down.
	 *
	 */
	TIM1->CR1 &= ~TIM_CR1_DIR;
	/* DIR: Direction
	 0: Counter used as upcounter
	 1: Counter used as downcounter
	 */
	TIM1->CR1 &= ~TIM_CR1_CKD;
	/*
	 CKD[1:0]: Clock division
	 This bit-field indicates the division ratio between the timer clock (CK_INT) frequency and the
	 dead-time and sampling clock (tDTS)used by the dead-time generators and the digital filters
	 (ETR, TIx):
	 00: tDTS=tCK_INT !!!
	 01: tDTS=2*tCK_INT
	 10: tDTS=4*tCK_INT
	 11: Reserved, do not program this value
	 Note: tDTS = 1/fDTS, tCK_INT = 1/fCK_INT.
	 */
	TIM1->ARR = 1000;
	TIM1->CCR1 = TIM1->ARR - PulseLengthUs;  // Довжина імпульса у мікросекундах
	TIM1->PSC = Prescaler; // Установка подільника тактування для таймера
	TIM1->RCR = PulseCount - 1;     // Установка кількості повторів імпульсів
	TIM1->EGR = TIM_EGR_UG; // Генерація події оновлення для негайної, перезагрузки PSC & RCR
	TIM1->CR1 |= TIM_CR1_OPM; // Вибiр режимy One Pulse Mode:
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2
			| TIM_CCMR1_OC1M_3 | TIM_CCMR1_OC1PE; // PWM Mode 1
	TIM1->SMCR = RESET;
	TIM1->CR1 |= TIM_CR1_ARPE; // Autoreload Preload Enable
	TIM1->CCER |= TIM_CCER_CC1E; // дозвіл виходу Compare каналу 1:
	TIM1->BDTR |= TIM_BDTR_MOE; // Дозвіл основного виходу таймера:

}
void Pulse() {
	TIM1->CR1 |= TIM_CR1_CEN; // Дозвіл роботи таймера

}

void TIM1_PWM_DMA_BRUST_Init(void) {

	/* TIM1 clock enable */
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	/* DMA1 clock enable */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	/* GPIOA clock enable */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Set CHSEL bits according to DMA Channel 5 */
	/* Set DIR bits according to Memory to peripheral direction */
	/* Set PINC bit according to DMA Peripheral Increment Disable */
	/* Set MINC bit according to DMA Memory Increment Enable */
	/* Set PSIZE bits according to Peripheral DataSize = Word */
	/* Set MSIZE bits according to Memory DataSize Word */
	/* Set CIRC bit according to circular mode */
	/* Set PL bits according to very high priority */
	/* Set MBURST bits according to single memory burst */
	/* Set PBURST bits according to single peripheral burst */
	DMA1_Channel5->CCR |= DMA_MEMORY_TO_PERIPH |
	DMA_PINC_DISABLE | DMA_MINC_ENABLE |
	DMA_PDATAALIGN_WORD | DMA_MDATAALIGN_WORD |
	DMA_CIRCULAR | DMA_PRIORITY_HIGH;
	/* Write to DMA1 Channel5 number of data register register */
	DMA1_Channel5->CNDTR = BUFFER_DATA_NUMBER;

	/* Write to DMA1 Channel5 peripheral address */
	DMA1_Channel5->CPAR = (uint32_t) TIM1_DMAR_ADDRESS;

	/* Write to DMA1 Channel5 Memory address register */
	DMA1_Channel5->CMAR = (uint32_t) aSRC_Buffer;

	/* Enable DMA1 Channel5 */
	DMA1_Channel5->CCR |= (uint32_t) DMA_CCR_EN;

	// Ініціалізація PA8, як канал 1 Таймера 1.
	GPIOA->MODER |= GPIO_MODER_MODER8_1;
	/*
	 00: Input mode (reset state)
	 01: General purpose output mode
	 10: Alternate function mode !!!
	 11: Analog mode
	 */
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
	/*
	 x0: Low speed
	 01: Medium speed
	 11: High speed !!!
	 */
	GPIOA->AFR[1] |= 0x00000006; /* AF6: TIM1_CH1*/
	Tim1Prescaler = (uint16_t) (SystemCoreClock / 32000000) - 1;
	/* Configure the period */
	TIM1->ARR = 0xFFFF;
	TIM1->PSC = Tim1Prescaler; // Встановлюємо подільник частоти таймера на 1 МГц
	TIM1->CCR1 = 0xFFF;
	TIM1->CR1 &= ~ TIM_CR1_CKD; // Подільник частоти таймера також на 1
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM Mode 1
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE; /* Enable the output compare 1 Preload */
	TIM1->CR1 |= TIM_CR1_ARPE; /* Enable auto-reload Preload  */
	TIM1->DIER |= TIM_DIER_UDE; /* Вмикаю оновлення таймера по DMA*/
	TIM1->DCR &= ~TIM_DCR_DBA;
	TIM1->DCR &= ~TIM_DCR_DBL;
	TIM1->DCR = TIM_DMABase_ARR | TIM_DMABurstLength_3Transfers; // Кількість порцій за одну DMA транзакцію
	TIM1->EGR |= TIM_EGR_UG; /* Обов'язково генеруємо подію Update */
	/* Wait until the RESET of UG bit*/
	while ((TIM1->EGR & TIM_EGR_UG) == SET)
		;
	/* Enable UEV by setting UG bit to load data from preload to active registers */
	TIM1->EGR |= TIM_EGR_UG;
	TIM1->BDTR |= TIM_BDTR_MOE; /*Вмикаю головний вихід TIM1*/
	TIM1->CCER |= TIM_CCER_CC1E;/*Вмикаю перший канал*/
	TIM1->CR1 |= TIM_CR1_CEN;
}

void InitTimerForInterruptGenerationMs(uint16_t TimeBaseMs) {
	/* GPIOA clock enable */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	/* Ініціалізація PA8, як GP_Output */
	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	/*
	 00: Input mode (reset state)
	 01: General purpose output mode !!!
	 10: Alternate function mode
	 11: Analog mode
	 */
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //TIM2 Timer clock enable
	TIM2->DIER |= TIM_DIER_UIE;   //Bit 0 UIE: Update interrupt enable
	TIM2->SMCR &= ~ TIM_SMCR_SMS;
	TIM2->CR1 |= TIM_CR1_CEN;   //Bit 0 CEN: Counter enable
	TIM2->PSC = SYSCLK / 1000 - 1;
	TIM2->ARR = TimeBaseMs;
	NVIC_EnableIRQ(TIM2_IRQn); //разрешить прерывания от таймера
}

void InitTimerForInterruptGenerationUs(uint16_t TimeBaseUs) {
	/* GPIOA clock enable */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	/* Ініціалізація PA8, як GP_Output */
	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	/*
	 00: Input mode (reset state)
	 01: General purpose output mode !!!
	 10: Alternate function mode
	 11: Analog mode
	 */
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //TIM2 Timer clock enable
	//TIM2->CR1 |= TIM_CR1_ARPE;  //Bit 7 ARPE: Auto-reload preload enable
	TIM2->DIER |= TIM_DIER_UIE;   //Bit 0 UIE: Update interrupt enable
	TIM2->SMCR &= ~TIM_SMCR_SMS;
	TIM2->CR1 |= TIM_CR1_CEN;   //Bit 0 CEN: Counter enable
	TIM2->PSC = SYSCLK / 1000000 - 1;
	TIM2->ARR = TimeBaseUs;

	NVIC_EnableIRQ(TIM2_IRQn); //разрешить прерывания от таймера

}

void TIM2_IRQHandler(void) { /* Обробник переривання з переповнення */
	GPIOA->ODR ^= GPIO_ODR_8;
	TIM2->SR &= ~ TIM_SR_UIF;
}

void ConfigCascadeTimersPulseBundleGeneration(void) {
	/* GPIOA clock enable */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	/* Ініціалізація PA8, як GP_Output */
	GPIOA->MODER |= GPIO_MODER_MODER8_1;
	/*
	 00: Input mode (reset state)
	 01: General purpose output mode
	 10: Alternate function mode!!!
	 11: Analog mode
	 */
	GPIOA->AFR[1] |= 0x00000006; /* AF6: TIM1_CH1*/
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
	/* Таймери конфігуруються наступним чином:
	 • TIM2 конфігурується в режимі master-тригера для подачі сигналу на TIM1.
	 • TIM1 конфігурується в режимі slave-запуску, і в режимі генерації одного імпульса (one pulse mode, OPM).
	 */


	/* Налаштування Slave таймера */
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // TIM1 clock enable
	uint16_t Tim1Prescaler, Period;
	Tim1Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1; // Конфігуруємо частоту так, щоб вона становила 1Мгц
	Period = 1000000 / 20000;

	TIM1->PSC = Tim1Prescaler;
	TIM1->ARR = Period - 1;
	TIM1->RCR = ((uint32_t) 3) - 1; // Кть імпульсів
	TIM1->CCR1 = Period / 2;        // Період імпульсів

	// Частота імпульсів у пачці: F = SYSCLK / ((PSC) * (ARR) * (RCR)), F = 8000000/((8)*(50)*(2)) = 10 кГц

	TIM1->CR1 &= ~ TIM_CR1_CKD;     // Подільник частоти 1

	TIM1->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); // Рахуємо вгору

	/* Очищаємо біотове поле режиму шим перед установкою */
	TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // PWM Mode 1

	/* One Pulse Mode */
	TIM1->CR1 |= TIM_CR1_OPM;

	// Вибір внутр трігера ETR1
	TIM1->SMCR &= ~TIM_SMCR_TS;
	TIM1->SMCR |= TIM_SMCR_TS_0; //001: Internal Trigger 1 (ITR1)


	TIM1->SMCR &= ~TIM_SMCR_SMS; // Вибір Slave Mode:
	TIM1->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1; // Trigger Mode
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE; // OC1PE: Output compare 1 preload enable

	TIM1->EGR |= TIM_EGR_UG;/*Reinitialize the counter and generates an update of the registers. The prescaler internal
	counter is also cleared (the prescaler ratio is not affected). The counter is cleared if the
	center-aligned mode is selected or if DIR=0 (upcounting), else it takes the auto-reload
	value (TIMx_ARR) if DIR=1 (downcounting)*/
	TIM1->BDTR |= TIM_BDTR_MOE; // Вмикаємо основний вихід TIM1:
	TIM1->CCER &= ~TIM_CCER_CC1P; // Reset Output Polarity:
	TIM1->CCER |= TIM_CCER_CC1P;  // Установка активного низького рівня
	TIM1->CCER |= TIM_CCER_CC1E;  // Активація виходу CH1
	TIM1->CR1 |= TIM_CR1_CEN;     // Активація модуля таймера


	/* Налаштування Master таймера */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	uint16_t Tim2Prescaler = 0;
	Tim2Prescaler = (uint16_t) ((SystemCoreClock) / 1000000) - 1;
	Period = 1000000 / 50; // Для періоду 20 мС і частоти генерації пачок імпульсів в 50 Гц)
	TIM2->ARR = Period - 1;
	TIM2->PSC = Tim2Prescaler;
	TIM2->CR1 &= ~ TIM_CR1_CKD; // clk div = 1
	TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); // Лічба вгору
	TIM1->CR2 &= ~ TIM_CR2_MMS;
	/*
	 000: Reset - the UG bit from the TIMx_EGR register is used as trigger output (TRGO). If the
	 reset is generated by the trigger input (slave mode controller configured in reset mode) then
	 the signal on TRGO is delayed compared to the actual reset
	 */
	TIM2->CR2 |= TIM_CR2_MMS_1;
	TIM2->CR1 |= TIM_CR1_CEN;

}
