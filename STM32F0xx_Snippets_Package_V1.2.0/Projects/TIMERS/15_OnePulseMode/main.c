/**
  ******************************************************************************
  * @file    15_OnePulseMode/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the timer to generate 
  *          a One Pulse Mode signal triggered by an externel signal. 
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
  - RCC
  - TIMx
  - GPIO PA15 and PB3 for TIM2_CH1 and TIM2_CH2
  - GPIO PC8 and PC9 for LEDs

 ===============================================================================
                    ##### How to use this example #####
 ===============================================================================
    - this file must be inserted in a project containing  the following files :
      o system_stm32f0xx.c, startup_stm32f072xb.s
      o stm32f0xx.h to get the register definitions
      o CMSIS files
 ===============================================================================
                    ##### How to test this example #####
 ===============================================================================
    - This example configures the TIM2 in order to generate a pulse 
      on OC1 (channel 1)with a period of 8 microseconds and delayed by 5 us 
      after a rising edge on IC2 (channel 2).
      The GPIO PA15, corresponding to TIM2_CH1, is configured as alternate function 
      and the AFR2 is selected.
      A Pulse is generated while a positive edge occurs on TIM2_CH2 
      i.e. GPIO PB3 configured as alternate function and the AFR2 is selected.
		- If PULSE_WITHOUT_DELAY is different from 0, a 8us long pulse is generate 
		  with a minimum delay (~66ns with a 48MHz clock) after the rising edge on IC2.
    - To test this example, the user must monitor the signal on PA15 and connect 
      a waveform generator on PB3.
    - This example can be easily ported on any other timer by modifying TIMx 
      definition. The corresponding GPIO configuration must be also adapted  
      according to the datasheet.
    - The green LED is switched on.

  *    
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
/** @addtogroup STM32F0_Snippets
  * @{
  */



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Define the Timer to be configured */
#define TIMx TIM2
#define TIMx_BASE TIM2_BASE

/* Define if the fast enable (OCxFE) must be set to generate a pulse without delay*/
#define PULSE_WITHOUT_DELAY 0

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void ConfigureTIMxAsOPM(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f072xb.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */
  ConfigureGPIO();
  ConfigureTIMxAsOPM();
  GPIOC->BSRR = 1<<9; /* switch on green led */
  while (1)  
  {  
    __WFI();
  }        
}

/**
  * @brief  This function enables the peripheral clocks on GPIO port C,
  *         configures GPIO PC9 in output mode for the Green LED pin,
  *         configures GPIO PC8 in output mode for the orange LED pin,
  * @param  None
  * @retval None
  */
__INLINE void ConfigureGPIO(void)
{  
  /* (1) Enable the peripheral clock of GPIOC */
  /* (2) Select output mode (01) on GPIOC pin 8 and 9 */
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; /* (1) */  
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8|GPIO_MODER_MODER9)) \
               | (GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0); /* (2) */  
}


/**
  * @brief  This function configures the TIMx as One Pulse Mode
  *         and enables the peripheral clock on TIMx and on GPIOA.
  *         It configures GPIO PA8 as Alternate function for TIM1_CH1, this must
  *         be modified if another timer than TIM1 is used or another channel
  *         according to the datasheet.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIMxAsOPM(void)
{
  /* (1) Enable the peripheral clock of Timer x */
  /* (2) Enable the peripheral clock of GPIOA and GPIOB*/
  /* (3) Select alternate function mode on GPIOA pin 15 */
  /* (4) Select AF2 on PA15 in AFRH for TIM2_CH1 */
  /* (5) Select alternate function mode on GPIOB pin 3 */
  /* (6) Select AF2 on PB3 in AFRL for TIM2_CH2 */
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* (1) */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN; /* (2) */  
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER15)) | (GPIO_MODER_MODER15_1); /* (3) */
  GPIOA->AFR[1] |= 0x02 << ((15 - 8) *4); /* (4) */
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER3)) | (GPIO_MODER_MODER3_1); /* (5) */
  GPIOB->AFR[0] |= 0x02 << (3 * 4); /* (6) */
  
  /* Use TI2FP2 as trigger 1 */
  /* (1) Map TI2FP2 on TI2 by writing CC2S=01 in the TIMx_CCMR1 register */
  /* (2) TI2FP2 must detect a rising edge, write CC2P=0 and CC2NP=0 
         in the TIMx_CCER register (keep the reset value) */
  /* (3) Configure TI2FP2 as trigger for the slave mode controller (TRGI) 
         by writing TS=110 in the TIMx_SMCR register 
         TI2FP2 is used to start the counter by writing SMS to ‘110' 
         in the TIMx_SMCR register (trigger mode) */
  TIMx->CCMR1 |= TIM_CCMR1_CC2S_0; /* (1) */
  //TIMx->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP); /* (2) */
  TIMx->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_1 
              | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1; /* (3) */
  
  /* The OPM waveform is defined by writing the compare registers */
  /* (1) Set prescaler to 47, so APBCLK/48 i.e 1MHz */ 
  /* (2) Set ARR = 7, as timer clock is 1MHz the period is 8 us */
  /* (3) Set CCRx = 5, the burst will be delayed for 5 us (must be > 0)*/
  /* (4) Select PWM mode 2 on OC1  (OC1M = 111),
         enable preload register on OC1 (OC1PE = 1, reset value) 
         enable fast enable (no delay) if PULSE_WITHOUT_DELAY is set*/
  /* (5) Select active high polarity on OC1 (CC1P = 0, reset value),
         enable the output on OC1 (CC1E = 1)*/
  /* (6) Enable output (MOE = 1)*/
  /* (7) Write '1 in the OPM bit in the TIMx_CR1 register to stop the counter 
         at the next update event (OPM = 1)
         enable auto-reload register(ARPE = 1) */  
  
  TIMx->PSC = 47; /* (1) */
  TIMx->ARR = 7; /* (2) */
  TIMx->CCR1 = 5; /* (3) */
  TIMx->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 
               | TIM_CCMR1_OC1PE 
#if PULSE_WITHOUT_DELAY > 0
               | TIM_CCMR1_OC1FE
#endif                 
               ; /* (4) */
  TIMx->CCER |= TIM_CCER_CC1E; /* (5) */
  TIMx->BDTR |= TIM_BDTR_MOE; /* (6) */
  TIMx->CR1 |= TIM_CR1_OPM | TIM_CR1_ARPE; /* (7) */
}


/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f072xb.s).                                               */
/******************************************************************************/



/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
