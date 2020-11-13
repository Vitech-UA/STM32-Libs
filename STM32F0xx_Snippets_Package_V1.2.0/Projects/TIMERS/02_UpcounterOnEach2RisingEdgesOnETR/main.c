/**
  ******************************************************************************
  * @file    02_UpcounterOnEach2RisingEdgesOnETR/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the TIMx ETR   
  *          to use it as external clock
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - TIM1
   - GPIO PA12 for TIM1 ETR
   - GPIO PC8 and PC9
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
      once two rising edges have occurred on its external trigger (ETR).
      This channel is used as external clock.
      The GPIO PA12, corresponding to TIM1_ETR, is configured as alternate function 
      and the AFR2 is selected.
      The solder bridge SB20 must be soldered to wire the connector pin 
      to the MCU pin.
      To test this example, the user button must be conected to PA12, this is 
      easily done by wiring PA0 to PA12 thru the connector pins.
      The TIM1 counter is loaded in a global variable named Counter which can be 
      monitored in live. The counter value is also used to blink the led a number
      of times equals to its value while the green led is on.
    - This example can be easily ported on another timer having the ETR feature
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
void ConfigureTIMxForETR(void);
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
  ConfigureTIMxForETR();
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
  * @brief  This function configures the TIMx  to count each 2 rising edges
  *         occuring on the ETR input.
  *         To use another timer, channel or GPIO, the 4 first code lines must 
  *         be adapted according to the datasheet.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIMxForETR(void)
{
  /* (1) Enable the peripheral clock of Timer 1 */
  /* (2) Enable the peripheral clock of GPIOA */
  /* (3) Select Alternate function mode (10) on GPIOA pin 12 */  
  /* (4) Select TIM1_ETR on PA12 by enabling AF2 for pin 12 in GPIOA AFRH register */ 
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (2) */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER12)) \
               | (GPIO_MODER_MODER12_1); /* (3) */ 
  GPIOA->AFR[1] |= 0x2 << ((12-8)*4); /* (4) */
  
  /* (1) As no filter is needed in this example, write ETF[3:0]=0000 
         in the TIMx_SMCR register. Keep the reset value. 
         Set the prescaler by writing ETPS[1:0]=01 in the TIMx_SMCR register. 
         Select rising edge detection on the ETR pin by writing ETP=0 
         in the TIMx_SMCR register. Keep the reset value.
         Enable external clock mode 2 by writing ECE=1 in the TIMx_SMCR register. */
  /* (2) Enable the counter by writing CEN=1 in the TIMx_CR1 register. */  
  TIMx->SMCR |= TIM_SMCR_ETPS_0 | TIM_SMCR_ECE; /* (1)  */
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
      Counter = TIMx->CNT;
      toggle ^= 1;
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
