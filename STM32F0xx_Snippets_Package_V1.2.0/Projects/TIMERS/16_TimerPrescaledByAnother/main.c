/**
  ******************************************************************************
  * @file    16_TimerPrescaledByAnother/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the Timer1 as prescaler   
  *          for Timer2
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - TIMx
   - TIMy
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
    - This example configures the TIM1 in master mode so that it outputs 
      a periodic trigger signal on each update event UEV.
      Timer 2 must be configured in slave mode using ITR1 as internal trigger.
      ITR1 is driven by Timer1.
      Timer2 is set in slave mode in external clock mode 1, so
      Timer 2 is clocked by the rising edge of the periodic Timer 1 trigger 
      signal.
      The TIM2 counter is loaded in a global variable named CounterSlave  
      which can be monitored in live.
      The green led toggles each time the CounterSlave changes, so each second.
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
volatile uint16_t CounterSlave = 0; // Contains the last value of slave counter
uint16_t CounterMaster = 0; // Contains the last value of master counter
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void ConfigureTIMxAsPrescalerForTIMy(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  uint16_t CounterSlaveTmp = 0;
  
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f072xb.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */
  ConfigureGPIO();
  ConfigureTIMxAsPrescalerForTIMy();
  
  while (1) /* Infinite loop */
  {
    CounterMaster = TIMx->CNT;
    CounterSlaveTmp = TIMy->CNT;
    if (CounterSlave != CounterSlaveTmp)
    {
      CounterSlave = CounterSlaveTmp;
      GPIOC->ODR ^= (1<<9); /* Toggle green led on PC9 */
    }   
  }
}

/**
  * @brief  This function enables the peripheral clocks on GPIO port C,
  *         configures GPIO PC9 in output mode for the Green LED pin,
  *         configures GPIO PC8 in output mode for the orange LED pin,
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureGPIO(void)
{  
  /* (1) Enable the peripheral clock of GPIOC */
  /* (2) Select output mode (01) on GPIOC pin 8 and 9 */
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; /* (1) */  
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8|GPIO_MODER_MODER9)) \
               | (GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0); /* (2) */  
}


/**
  * @brief  This function configures the TIMx to be the prescaler for TIMy.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIMxAsPrescalerForTIMy(void)
{
  /* (1) Enable the peripheral clock of Timer x */
  /* (2) Enable the peripheral clock of Timer y */
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* (2) */
  
  /* (1) Select Update Event as Trigger output (TRG0) by writing MMS = 010 
         in TIMx_CR2. */
  /* (2) Configure TIMy in slave mode using ITR1 as internal trigger 
         by writing TS = 000 in TIMy_SMCR (reset value)
         Configure TIMy in external clock mode 1, by writing SMS=111 in the
         TIMy_SMCR register. */
  /* (3) Set TIMx prescaler to 47999 in order to get an increment each 1ms */
  /* (4) Set TIMx Autoreload to 999 in order to get an overflow (so an UEV) 
         each second */
  /* (5) Set TIMx Autoreload to 24*3600-1 in order to get an overflow each 24-hour */
  /* (6) Enable the counter by writing CEN=1 in the TIMx_CR1 register. */  
  /* (7) Enable the counter by writing CEN=1 in the TIMy_CR1 register. */    
  TIMx->CR2 |= TIM_CR2_MMS_1; /* (1)*/  
  TIMy->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0; /* (2) */
  TIMx->PSC = 47999; /* (3) */
  TIMx->ARR = 999; /* (4) */
  TIMy->ARR = (24 * 3600) - 1; /* (5) */
  TIMx->CR1 |= TIM_CR1_CEN; /* (6) */  
  TIMy->CR1 |= TIM_CR1_CEN; /* (7) */
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
