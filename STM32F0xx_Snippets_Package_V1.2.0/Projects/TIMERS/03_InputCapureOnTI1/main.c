/**
  ******************************************************************************
  * @file    03_InputCaptureOnTI1/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the timer to capture 
  *          the counter value in TIMx_CCR1 when TI1 input rises.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
    - RCC
    - TIMx
    - GPIO PA8 for TIM1_CH1
    - GPIO PC8,9 for LEDs
    - SYSTICK (to manage led blinking)

 ===============================================================================
                    ##### How to use this example #####
 ===============================================================================
    - this file must be inserted in a project containing the following files:
      o system_stm32f0xx.c, startup_stm32f072xb.s
      o stm32f0xx.h to get the register definitions
      o CMSIS files
 ===============================================================================
                   ##### How to test this example #####
 ===============================================================================
    - This example configures the TIM1 in order to compute the elapsed time 
      between two rising edges occurring on its TI1 (channel 1).
      This channel is used as trigger input.
      The TIMx interrupt subroutine computes the period of the signal on 
      TIMx IC1F.
      The GPIO PA8, corresponding to TIM1_CH1, is configured as alternate function 
      and the AFR2 is selected.
      To test this example, the user button can be connected to PA8, this is 
      easily done by wiring PA0 to PA8 thru the connector pins or by connecting 
      a wave generator to PA8.
      The elapsed time is loaded in a global variable named Counter which can be 
      monitored in live.
      The maximum elapsed time which can be measured between two pulses 
      is 700 ms with these settings RCC and TIMx. To remove this limitation, 
      the Update interrupt must be implemented.
    - This example can be easily ported on any other timer by modifying TIMx 
      definition. The corresponding GPIO must be also adapted according to 
      the datasheet.
    - The green LED lit once a period has been computed.

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
/* Delay value : short one is used for the error coding, long one in case of no error
   or between two bursts */
#define SHORT_DELAY 200
#define LONG_DELAY 1000

#define TIMx TIM1
#define TIMx_BASE TIM1_BASE
#define TIMx_IRQn TIM1_CC_IRQn
#define TIMx_IRQHandler TIM1_CC_IRQHandler

/* Error codes used to make the orange led blinking */
#define ERROR_WRONG_IT 0x01


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t error = 0xFF;  //initialized at 0xFF and set in the TIM14 interrupt subroutine
uint16_t gap = 0;    //initialized at 0 and used to synchronize the IC laps computation
uint16_t counter0; //used to store the first rising edge IC counter value 
uint16_t Counter;
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void ConfigureTIMxAsInputCapture(void);
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
  ConfigureTIMxAsInputCapture();
    
  while ((error == 0) || (error == 0xFF)) 
  {  
     __WFI();
     if (error == 0)
     {
       GPIOC->BSRR = 1<<9; /* switch on green led */
     }
     else
     {
       GPIOC->BRR = 1<<9; /* switch off green led */
     }       
  }
  SysTick_Config(750); /* 1ms config */
  while (1) /* Infinite loop */
  {
  }    
}


/**
  * @brief  This function enables the peripheral clock on GPIO port C,
  *         configures the Green LED pin on GPIO PC9,
  *         configures the orange LED pin on GPIO PC8.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureGPIO(void)
{
  /* (1) Enable the peripheral clock of GPIOC */
  /* (2) Select output mode (01) on GPIOC pin 8 and 9 */
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; /* (1) */
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9)) \
               | (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0); /* (2) */
}


/**
  * @brief  This function configures the TIMx as input capture 
  *         and enables the interrupt on TIMx. It also enables the peripheral
  *         clock on TIMx and on GPIOA, set the PCLK and HCLK to get 
  *         one count each 10.66us.
  *         It configures GPIO PA8 as Alternate function for TIM1_CH1
  *         To use another timer, channel or GPIO, the RCC and GPIO configuration 
  *         must be adapted according to the datasheet.
  *         In case of other timer, the interrupt sub-routine must also be renamed
  *         with the right handler and the NVIC configured correctly.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIMxAsInputCapture(void)
{
  /* Configure NVIC for TIMx */
  /* (1) Enable Interrupt on TIMx */
  /* (2) Set priority for TIMx*/
  NVIC_EnableIRQ(TIMx_IRQn); /* (1) */
  NVIC_SetPriority(TIMx_IRQn,0); /* (2) */
  
  /* (1) Enable the peripheral clock of Timer x */
  /* (2) Set PCLK clock prescaler to /16 (111)
         set HCLK clock prescaler to /64 (1100) */
  /* (3) Enable the peripheral clock of GPIOA */
  /* (4) Select alternate function mode on GPIOA pin 8 */
  /* (5) Select AF2 on PA8 in AFRH for TIM1_CH1 */
  

  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */
  RCC->CFGR |= RCC_CFGR_PPRE | RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2; /* (2) */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (3) */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER8)) | (GPIO_MODER_MODER8_1); /* (4) */
  GPIOA->AFR[1] |= 0x02; /* (5) */
  
  /* (1) Select the active input TI1 (CC1S = 01),
         program the input filter for 8 clock cycles (IC1F = 0011), 
         select the rising edge on CC1 (CC1P = 0, reset value)
         and prescaler at each valid transition (IC1PS = 00, reset value) */
  /* (2) Enable capture by setting CC1E */
  /* (3) Enable interrupt on Capture/Compare */
  /* (4) Enable counter */  
  
  TIMx->CCMR1 |= TIM_CCMR1_CC1S_0 \
               | TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1; /* (1)*/
  TIMx->CCER |= TIM_CCER_CC1E; /* (2) */  
  TIMx->DIER |= TIM_DIER_CC1IE; /* (3) */
  TIMx->CR1 |= TIM_CR1_CEN; /* (4) */
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
  *         It toggles the green led if the action has been performed correctly
  *         and toggles the orange led coding the error number
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  static uint32_t long_counter = LONG_DELAY;
  static uint32_t short_counter = SHORT_DELAY;  
  static uint16_t error_temp = 0;
  
  if (long_counter-- == 0) 
  {
    if(error == 0)
    {
      /* the following instruction can only be used if no ISR modifies GPIOC ODR
         either by writing directly it or by using GPIOC BSRR or BRR 
         else a toggle mechanism must be implemented using GPIOC BSRR and/or BRR
      */
      GPIOC->ODR ^= (1<<9);//toggle green led on PC9
      long_counter = LONG_DELAY;
    }
    else if (error != 0xFF)
    {
      /* orange led blinks according to the code error value */
      error_temp = (error << 1) - 1;
      short_counter = SHORT_DELAY;
      long_counter = LONG_DELAY << 1;
      GPIOC->BSRR = (1<<8); //set orange led on PC8
      GPIOC->BRR = (1<<9); //switch off green led on PC9
    }
  }
  if (error_temp > 0)
  {
    if (short_counter-- == 0) 
    {
      GPIOC->ODR ^= (1 << 8); //toggle orange led
      short_counter = SHORT_DELAY;
      error_temp--;
    }  
  }
}


/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f072xb.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles TIM1 interrupt request.
  *         This interrupt subroutine computes the laps between 2 rising edges 
  *         on T1IC. This laps is stored in the "Counter" variable.
  * @param  None
  * @retval None
  */
void TIM1_CC_IRQHandler(void)
{
uint16_t counter1;
  
/*
  
*/
  if ((TIMx->SR & TIM_SR_CC1IF) != 0)
  {
    if ((TIMx->SR & TIM_SR_CC1OF) != 0)  /* Check the overflow */
    {
      error = 0xFF;
      gap = 0;  /* Reinitialize the laps computing */
      TIMx->SR &= ~(TIM_SR_CC1OF | TIM_SR_CC1IF); /* Clear the flags */
      return;
    }
    if (gap == 0) /* Test if it is the first rising edge */
    {
      counter0 = TIMx->CCR1; /* Read the capture counter which clears the CC1ICF */
      gap = 1; /* Indicate that the first rising edge has yet been detected */
    }
    else
    {
      counter1 = TIMx->CCR1; /* Read the capture counter which clears the CC1ICF */
      if (counter1 > counter0) /* Check capture counter overflow */
      {
        Counter = counter1 - counter0;
      }
      else
      {
        Counter = counter1 + 0xFFFF - counter0 + 1;
      }
      counter0 = counter1;
      error = 0;
    }    
  }
  else
  {
    error = ERROR_WRONG_IT; /* Report an error */
  }
}
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
