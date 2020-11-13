/**
  ******************************************************************************
  * @file    14_SlaveExtClkMode2andTriggerMode/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the TIMx Channel 1   
  *          to use it to trigger the counter and counting on ETR rising edge.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - TIMx
   - GPIO PA8 and PA12 for TIM1_CH1 and TIM_ETR
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
    - This example configures the TIM1 in order to start the counter in response 
      to a rising edge on TI1 input and counts the rising edges on the TIM1_ETR.
      The GPIO PA8 and PA12, corresponding to TIM1_CH1 and TIM1_ETR, are 
      configured as alternate function and the AFR2 is selected.
    -  To test this example : 
        - solder SB20
        - wire between PA0 and PA8
        - Function generator on PA12 (square signal,
          between 100Hz and 10kHz to see the LED blinking properly)
        - Load the code with IDE
        - add the variable "Counter" in a watch window
        - Run the program
        - Press the button, the counter start
        - The LED toggles each time the counter value is a multiple of 0x80,
          this allows to check the counter is started.
    - This example can be easily ported on another timer getting the trigger 
      mode and supporting external clock.
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
#define TIMx_BASE       TIM1_BASE
#define TIMx            ((TIM_TypeDef *) TIMx_BASE)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t Counter = 0; // Contains the last value of the captured counter
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void ConfigureTIMxAsExtClkInTriggerMode(void);
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
  ConfigureTIMxAsExtClkInTriggerMode();
  while (1) /* Infinite loop */
  {
    Counter = TIMx->CNT;
    if ((Counter != 0) && ((Counter & ~(0xFF80)) == 0))
    {
      /* the following instruction can only be used if no ISR modifies GPIOC ODR
         either by writing directly it or by using GPIOC BSRR or BRR 
         else a toggle mechanism must be implemented using GPIOC BSRR and/or BRR
      */
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
  * @brief  This function configures the TIMx in Trigger mode to count once the 
  *         a rising edge occurs on TI1. The timer counter is enabled by HW.
  *         The timer counts the rising edges on the TIM_ETR pin.
  *         To use another timer or GPIO, the RCC and GPIO configuration 
  *         must be adapted according to the datasheet. 
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIMxAsExtClkInTriggerMode(void)
{
  /* (1) Enable the peripheral clock of Timer x */
  /* (2) Enable the peripheral clock of GPIOA */
  /* (3) Select Alternate function mode (10) on GPIOA pins 8 and 12 */  
  /* (4) Select TIM1_CH1 on PA8 and TIM_ETR on PA12 by enabling AF2 for pin 8 
         and 12 in GPIOA AFRH register */ 

  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (2) */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER12)) \
               | (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER12_1); /* (3) */  
  GPIOA->AFR[1] |= 0x02 | (0x2 << ((12-8)*4)) ; /* (4) */
  
  /* (1) Configure no input filter (ETF=0000, reset value)
         configure prescaler disabled (ETPS = 0, reset value)
         select detection on rising edge on ETR (ETP = 0, reset value)
         enable external clock mode 2 (ECE = 1)*/
  /* (2) Configure no input filter (IC1F=0000, reset value)
         select input capture source on TI1 (CC1S = 01) */
  /* (3) Select polarity by writing CC1P=0 (reset value) in the TIMx_CCER 
         register */
  /* (4) Configure the timer in trigger mode by writing SMS=110 
         Select TI1 as the trigger input source by writing TS=101 
         in the TIMx_SMCR register.*/
  TIMx->SMCR |= TIM_SMCR_ECE; /* (1) */
  TIMx->CCMR1 |= TIM_CCMR1_CC1S_0; /* (2)*/  
  //TIMx->CCER &= ~TIM_CCER_CC1P; /* (3) */
  TIMx->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 \
              | TIM_SMCR_TS_2 | TIM_SMCR_TS_0; /* (4) */
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
