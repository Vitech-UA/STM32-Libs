/**
  ******************************************************************************
  * @file    19_TwoTimersSynchronizedByExtTrig/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to synchronize two timers   
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - TIMx
   - TIMy
   - GPIO PA5 and PA8, and PA9 for TIM2_CH1, TIM1_CH1(TI1) and TIM1_CH2

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
    - This example configures the TIM1 and TIM2 in order to start them 
      synchronously on an external trigger on TI1 (TIM1_CH1).
      Timer 1 is in master/slave mode with CEN used as Trigger output.
      Timer 2 must be configured in slave mode using ITR1 as internal trigger.     
      Timer2 is set in slave mode in trigger mode.
      To visualize the behaviour, TIM1_CH2 is configured in PWM mode 1 and 
      TIM2_CH1 also.
      TIM1_CH2 and TIM2_CH1 are configured with the same PWM.
      TIM1_CH1(PA8) must be wired to PA0, so a push on the USER button will 
      start both timers.
      To monitor the signals, use an oscilloscope on PA9 for TIM1_CH1 and PA5
      for TIM2_CH1.
      Due to PA5 characteristics, the slope are not as fast as on PA9.
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
void ConfigureTIMxSynchroWithTIMy(void);
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
  ConfigureTIMxSynchroWithTIMy();
  while (1) /* Infinite loop */
  {
    __WFI();   
  }
}


/**
  * @brief  This function configures the TIMx and TIMy to be started 
  *         synchronously.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIMxSynchroWithTIMy(void)
{
  /* (1) Enable the peripheral clock of Timer x */
  /* (2) Enable the peripheral clock of Timer y */
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* (2) */

  /* (1) Enable the peripheral clock of GPIOA */
  /* (2) Select alternate function mode on GPIOA pin 5, 8 and 9 */
  /* (3) Select AF2 on PA5 in AFRL for TIM2_CH1 */
  /* (4) Select AF2 on PA8 and PA9 in AFRH for TIM1_CH1 and TIM1_CH2 */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (1) */  
  GPIOA->MODER = (GPIOA->MODER 
              & ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER8 | GPIO_MODER_MODER9)) 
               | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1; /* (2) */
  GPIOA->AFR[0] |= 0x02 << (5 * 4); /* (3) */
  GPIOA->AFR[1] |= 0x02 | (0x02 << ((9 - 8) * 4)); /* (4) */
  
  /* (1) Configure TIMx master mode to send its enable signal 
         as trigger output (MMS=001 in the TIM1_CR2 register). */
  /* (2) Configure TIMx in slave mode to get the input trigger from TI1
         by writing TS = 100 in TIMx_SMCR
         Configure TIMx in trigger mode, by writing SMS=110 in the
         TIMx_SMCR register.
         Configure TIMx in Master/Slave mode by writing MSM = 1 in TIMx_SMCR */
  /* (3) Configure TIMy in slave mode to get the input trigger from Timer1 
         by writing TS = 000 in TIMy_SMCR (reset value)
         Configure TIMy in trigger mode, by writing SMS=110 in the
         TIMy_SMCR register. */
  /* (4) Reset Timer x counter by writing ‘1 in UG bit (TIMx_EGR register) */
  /* (5) Reset Timer y counter by writing ‘1 in UG bit (TIMy_EGR register) */  
  TIMx->CR2 |= TIM_CR2_MMS_0; /* (1)*/ 
  TIMx->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1
              | TIM_SMCR_MSM; /* (2) */
  TIMy->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1; /* (3) */
  TIMx->EGR |= TIM_EGR_UG; /* (4) */
  TIMy->EGR |= TIM_EGR_UG; /* (5) */
  
  /* Configure the Timer Channel 2 as PWM as PWM  */
  /* (1) Configure the Timer 1 Channel 2 waveform (TIM1_CCMR1 register)
         is in PWM mode 1 (write OC2M = 110) */
  /* (2) Set TIMx prescaler to 2 */
  /* (3) Set TIMx Autoreload to 99 in order to get an overflow (so an UEV) 
         each 6.2µs */
  /* (4) Set capture compare register to a value between 0 and 99 */
  TIMx->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; /* (1) */
  TIMx->PSC = 2; /* (2) */
  TIMx->ARR = 99; /* (3) */
  TIMx-> CCR2 = 25; /* (4) */
  /* Configure the slave timer Channel 1 as PWM as Timer to show synchronicity  */
  /* (1) Configure the Timer 2 in PWM mode 1 (write OC1M = 110) */
  /* (2) Set TIMy prescaler to 2 */
  /* (3) Set TIMx Autoreload to 99  */
  /* (4) Set capture compare register to 25 */
  TIMy->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; /* (1) */
  TIMy->PSC = 2; /* (2) */
  TIMy->ARR = 99; /* (2) */
  TIMy-> CCR1 = 25; /* (3) */
  /* Enable the output of TIMx OC1 */
  /* (1) Select active high polarity on OC1 (CC1P = 0, reset value),
         enable the output on OC1 (CC1E = 1)*/
  /* (2) Enable output (MOE = 1)*/
  TIMx->CCER |= TIM_CCER_CC2E; /* (1) */
  TIMx->BDTR |= TIM_BDTR_MOE; /* (2) */
  /* Enable the output of TIMy OC1 */
  /* (1) Select active high polarity on OC1 (CC1P = 0, reset value),
         enable the output on OC1 (CC1E = 1)*/
  /* (2) Enable output (MOE = 1)*/
  TIMy->CCER |= TIM_CCER_CC1E; /* (1) */
  TIMy->BDTR |= TIM_BDTR_MOE; /* (2) */
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
