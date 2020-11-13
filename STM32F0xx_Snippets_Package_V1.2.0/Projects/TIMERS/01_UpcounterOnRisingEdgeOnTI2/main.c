/**
  ******************************************************************************
  * @file    01_UpcounterOnRisingEdgeOnTI2/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the TIMx Channel 2   
  *          to use it as external clock
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - TIMx
   - GPIO PA9 for TIM1_CH2
   - GPIO PC8 and PC9 for LEDs
   - SYSTICK (to manage led blinking)

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
    - This example configures the TIM1 in order to increment the counter
      while a rising edge occurs on its channel 2.
      This channel is used as external clock.
      The GPIO PA9, corresponding to TIM1_CH2, is configured as alternate function 
      and the AFR2 is selected.
      To test this example, the user button must be connected to PA9, this is 
      easily done by wiring PA0 to PA9 thru the connector pins.
      The TIM1 counter is loaded in a global variable named Counter which can be 
      monitored in live. The counter value is also used to blink the led a number
      of times equals to its value while the green led is on.
    - This example can be easily ported on another timer getting a Channel 2 
      by modifying TIMx definition.
      
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
/* Delay value : short one is used for the error coding, long one (~1s) in case 
   of no error or between two bursts */
#define SHORT_DELAY 100
#define LONG_DELAY 2000


/* Define the Timer to be configured */
#define TIMx_BASE       TIM1_BASE
#define TIMx            ((TIM_TypeDef *) TIMx_BASE)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t error = 0;  //initialized at 0 and modified by the functions 

uint16_t Counter = 0; // Contains the last value of the captured counter
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void ConfigureTIMxAsUpcounter(void);
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
  SysTick_Config(48000);/* 1ms config */  
  ConfigureTIMxAsUpcounter();
  while (1) /* Infinite loop */
  {
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
  * @brief  This function configures the TIMx to perform the count of rising 
  *         edges on TI2.
  *         To use another timer, channel or GPIO, the 4 first code lines must 
  *         be adapted according to the datasheet.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIMxAsUpcounter(void)
{
  /* (1) Enable the peripheral clock of Timer 1 */
  /* (2) Enable the peripheral clock of GPIOA */
  /* (3) Select Alternate function mode (10) on GPIOA pin 9 */  
  /* (4) Select TIM1_CH2 on PA9 by enabling AF2 for pin 9 in GPIOA AFRH register */ 

  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (2) */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9)) \
               | (GPIO_MODER_MODER9_1); /* (3) */  
  GPIOA->AFR[1] |= 0x2 << ((9-8)*4); /* (4) */
  
  /* (1) Configure channel 2 to detect rising edges on the TI2 input 
         by writing CC2S = ‘01’, 
         and configure the input filter duration by writing the IC2F[3:0] bits 
         in the TIMx_CCMR1 register (if no filter is needed, keep IC2F=0000).*/
  /* (2) Select rising edge polarity by writing CC2P=0 in the TIMx_CCER register 
         Not necessary as it keeps the reset value. */
  /* (3) Configure the timer in external clock mode 1 by writing SMS=111 
         Select TI2 as the trigger input source by writing TS=110 
         in the TIMx_SMCR register.*/
  /* (4) Enable the counter by writing CEN=1 in the TIMx_CR1 register. */  
  TIMx->CCMR1 |= TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_CC2S_0; /* (1)*/  
  //TIMx->CCER &= (uint16_t)(~TIM_CCER_CC2P); /* (2) */
  TIMx->SMCR |= TIM_SMCR_SMS | TIM_SMCR_TS_2 | TIM_SMCR_TS_1; /* (3) */
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
  *         and toggles the orange led coding the error number or information
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  static uint32_t long_counter = LONG_DELAY;
  static uint32_t short_counter = SHORT_DELAY;  
  static uint16_t error_temp = 0;
  static uint8_t toggle = 0;
  
  if (long_counter-- == 0) 
  {
    if(error == 0)
    {
      toggle ^= 1;
      Counter = TIMx->CNT;
      if (toggle == 0)
      {
        GPIOC->BSRR = (1<<9); //switch on green led on PC9
        if (Counter > 0)
        {
          error_temp = (Counter << 1) - 1;
          short_counter = SHORT_DELAY;
          GPIOC->BSRR = (1<<8); //set orange led on PC8
        }       
      }
      else
      {
        GPIOC->BRR = (1<<9); //switch off green led on PC9
      }
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
