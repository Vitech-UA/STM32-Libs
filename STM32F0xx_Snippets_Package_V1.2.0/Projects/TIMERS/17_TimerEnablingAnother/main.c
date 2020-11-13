/**
  ******************************************************************************
  * @file    17_TimerEnablingAnother/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the TIMx to enable TIMy   
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - TIMx
   - TIMy
   - GPIO PA5 and PA8 for TIM2_CH1 and TIM1_CH1

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
    - This example configures the TIM1 in master mode so that it enables the 
      slave timer when OC1REF is active.
      Timer 2 must be configured in slave mode using ITR1 as internal trigger.     
      Timer2 is set in slave mode in external clock mode 1, so
      Timer 2 is in gated mode.
      To visualize the behaviour, TIM1_CH1 is configured in PWM mode 1 and 
      TIM2_CH1 also.
      TIM1_CH1 PWM has a higher period than TIM2_CH1 which toggles at each count.
      To monitor the signals, use an oscilloscope on PA8 for TIM1_CH1 and PA5
      for TIM2_CH1.
    - This example can be easily ported on other timers getting slave 
      and/or master features.
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


/* Define the Timers to be configured */
#define TIMx_BASE      TIM1_BASE
#define TIMy_BASE      TIM2_BASE
#define TIMx           ((TIM_TypeDef *) TIMx_BASE)
#define TIMy           ((TIM_TypeDef *) TIMy_BASE)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void ConfigureTIMxToEnableTIMy(void);
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
  ConfigureTIMxToEnableTIMy();
  while (1) /* Infinite loop */
  {
    __WFI();   
  }
}


/**
  * @brief  This function configures the TIMx to enable counting by TIMy.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIMxToEnableTIMy(void)
{
  /* (1) Enable the peripheral clock of Timer x */
  /* (2) Enable the peripheral clock of Timer y */
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* (2) */

  /* (1) Enable the peripheral clock of GPIOA */
  /* (2) Select alternate function mode on GPIOA pin 5 and 8 */
  /* (3) Select high speed for GPIOA pin 5 (max 50MHz) - OSPEEDR5 = 11 */
  /* (4) Select AF2 on PA5 in AFRL for TIM2_CH1 */
  /* (5) Select AF2 on PA8 in AFRH for TIM1_CH1 */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (1) */  
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER8)) 
               | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER8_1; /* (2) */
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5; /* (3) */
  GPIOA->AFR[0] |= 0x02 << (5 * 4); /* (4) */
  GPIOA->AFR[1] |= 0x02; /* (5) */
  
  /* (1) Configure Timer 1 master mode to send its Output Compare 1 Reference (OC1REF)
         signal as trigger output (MMS=100 in the TIM1_CR2 register). */
  /* (2) Configure the Timer 1 OC1REF waveform (TIM1_CCMR1 register)
         Channel 1 is in PWM mode 1 when the counter is less than the capture/compare 
         register (write OC1M = 110) */
  /* (3) Configure TIMy in slave mode using ITR1 as internal trigger 
         by writing TS = 000 in TIMy_SMCR (reset value)
         Configure TIMy in gated mode, by writing SMS=101 in the
         TIMy_SMCR register. */
  /* (4) Set TIMx prescaler to 2 */
  /* (5) Set TIMy prescaler to 2 */
  /* (6) Set TIMx Autoreload to 999 in order to get an overflow (so an UEV) 
         each 62µs */
  /* (7) Set capture compare register to a value between 0 and 999 */
  TIMx->CR2 |= TIM_CR2_MMS_2; /* (1)*/  
  TIMx->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; /* (2) */
  TIMy->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0; /* (3) */
  TIMx->PSC = 2; /* (4) */
  TIMy->PSC = 2; /* (5) */  
  TIMx->ARR = 999; /* (6) */
  TIMx-> CCR1 = 700; /* (7) */
  /* Configure the slave timer to generate toggling on each count */
  /* (1) Configure the Timer 2 in PWM mode 1 (write OC1M = 110) */
  /* (2) Set TIMx Autoreload to 1  */
  /* (3) Set capture compare register to 1 */
  TIMy->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; /* (1) */
  TIMy->ARR = 1; /* (2) */
  TIMy-> CCR1 = 1; /* (3) */
  /* Enable the output of TIMx OC1 */
  /* (1) Select active high polarity on OC1 (CC1P = 0, reset value),
         enable the output on OC1 (CC1E = 1)*/
  /* (2) Enable output (MOE = 1)*/
  TIMx->CCER |= TIM_CCER_CC1E; /* (1) */
  TIMx->BDTR |= TIM_BDTR_MOE; /* (2) */
  /* Enable the output of TIMy OC1 */
  /* (1) Select active high polarity on OC1 (CC1P = 0, reset value),
         enable the output on OC1 (CC1E = 1)*/
  /* (2) Enable output (MOE = 1)*/
  TIMy->CCER |= TIM_CCER_CC1E; /* (1) */
  TIMy->BDTR |= TIM_BDTR_MOE; /* (2) */
  /* (1) Enable the slave counter first by writing CEN=1 in the TIMy_CR1 register. */  
  /* (2) Enable the master counter by writing CEN=1 in the TIMx_CR1 register. */      
  TIMy->CR1 |= TIM_CR1_CEN; /* (1) */  
  TIMx->CR1 |= TIM_CR1_CEN; /* (2) */
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
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*
void PPP_IRQHandler(void)
{
}
*/

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
