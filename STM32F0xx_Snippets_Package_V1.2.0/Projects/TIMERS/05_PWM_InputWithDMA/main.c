/**
  ******************************************************************************
  * @file    05_PWM_InputWithDMA/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the timer in PWM input mode
  *          using the DMA to get the result without CPU load. 
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
  - TIMx
  - GPIO PA8 for TIM1_CH1
  - GPIO PC8 and PC9 for LEDs
  - DMA
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
    - This example configures the TIM1 in order to compute the period and the  
      duty cycle of a signal applied on its TI1 (channel 1).
      This channel is used as trigger input.
      The period and duty cycle of the signal on TIMx CH1 are downloaded 
      by the DMA from respectively the CCR1 and CCR2.
      The GPIO PA8, corresponding to TIM1_CH1, is configured as alternate function 
      and the AFR2 is selected.
    - To test this example, the user must applied a periodic signal on PA8.
      The period and duty cycle are loaded in global variables named Period and
      DutyCycle which can be monitored in live.
    - This example can be easily ported on any other timer by modifying TIMx 
      definition. The corresponding GPIO must be also configured as alternate 
      function according to the datasheet.
    - The green LED toggles if the period is not null and the duty cycle
      less than the period and till no fatal error is detected.
      In case of capture overflow, the orange led is switched on each time 
      an error is reported.

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
#define SHORT_DELAY 200
#define LONG_DELAY 1000


/* Define the Timer to be configured */
#define TIMx TIM1
#define TIMx_BASE TIM1_BASE
#define TIMx_IRQn TIM1_CC_IRQn
#define TIMx_IRQHandler TIM1_CC_IRQHandler

/* Error codes used to make the orange led blinking */
#define ERROR_DMA_XFER 0x01
#define ERROR_UNEXPECTED_DMA_IT 0x02

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t error = 0;  // Initialized at 0 and modified by the functions 
uint16_t Period = 0; // Period of the signal applied on TI1 based on 48MHz clock
uint16_t DutyCycle = 0; // High signal time based on 48MHz clock
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void ConfigureTIMxAsPWM_Input(void);
void ConfigureDMA(void);
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
  ConfigureTIMxAsPWM_Input();
  ConfigureDMA();
  GPIOC->BSRR = 1<<9; /* switch on green led */
  while (error < ERROR_UNEXPECTED_DMA_IT)  
  {  
    __WFI();
  }
  GPIOC->BRR = 1<<9; /* switch off green led */  
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
  * @brief  This function configures the TIMx as PWM input with a DMA transfer.
  *         It also enables the peripheral clock on TIMx and on GPIOA.
  *         It configures GPIO PA8 as Alternate function for TIM1_CH1
  *         To use another timer, channel or GPIO, the RCC and GPIO configuration 
  *         must be adapted according to the datasheet.
  *         In case of other timer, the interrupt sub-routine must also be renamed
  *         with the right handler and the NVIC configured correctly.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIMxAsPWM_Input(void)
{
  /* (1) Enable the peripheral clock of Timer x */
  /* (2) Enable the peripheral clock of GPIOA */
  /* (3) Select alternate function mode on GPIOA pin 8 */
  /* (4) Select AF2 on PA8 in AFRH for TIM1_CH1 */
  
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (2) */  
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER8)) | (GPIO_MODER_MODER8_1); /* (3) */
  GPIOA->AFR[1] |= 0x02; /* (4) */
  
  /* (1) Select the active input TI1 for TIMx_CCR1 (CC1S = 01), 
         select the active input TI1 for TIMx_CCR2 (CC2S = 10) */ 
  /* (2) Select TI1FP1 as valid trigger input (TS = 101)
         configure the slave mode in reset mode (SMS = 100) */
  /* (3) Enable capture by setting CC1E and CC2E 
         select the rising edge on CC1 and CC1N (CC1P = 0 and CC1NP = 0, reset value),
         select the falling edge on CC2 (CC2P = 1). */
  /* (4) Enable DMA request on Capture/Compare 1 and 2 */
  /* (5) Enable counter */  
  
  TIMx->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1; /* (1)*/
  TIMx->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_0 \
              | TIM_SMCR_SMS_2; /* (2) */
  TIMx->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P; /* (3) */  
  TIMx->DIER |= TIM_DIER_CC1DE | TIM_DIER_CC2DE; /* (4) */
  TIMx->CR1 |= TIM_CR1_CEN; /* (5) */
}


/**
  * @brief  This function configures the DMA to load values from TIM1 CCR1 and CCR2 
  *         to the memory.
  *         The load is performed at TIM1 request.
  *         TIM CH1 (CCR1) is connected to DMA channel 2 and TIM1 CH2 (CCR2) to
  *         DMA channel 3.
  *         The DMA is configured in circular mode and read from memory to peripheral. 
  *         Only the transfer error can trigger an interrupt.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureDMA(void)
{
  /* (1) Enable the peripheral clock on DMA */ 
  /* (2) Configure the peripheral data register address for DMA channel x */
  /* (3) Configure the memory address for DMA channel x */
  /* (4) Configure the number of DMA tranfer to be performed on DMA channel x */
  /* (5) Configure no increment (reset value), size (16-bits), interrupts,  
         transfer from peripheral to memory and circular mode  for DMA channel x */
  /* (6) Enable DMA Channel x */
  RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* (1) */
  DMA1_Channel2->CPAR = (uint32_t) (&(TIM1->CCR1)); /* (2) */
  DMA1_Channel2->CMAR = (uint32_t)(&Period); /* (3) */
  DMA1_Channel2->CNDTR = 1; /* (4) */
  DMA1_Channel2->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 \
                      | DMA_CCR_TEIE | DMA_CCR_CIRC; /* (5) */   
  DMA1_Channel2->CCR |= DMA_CCR_EN; /* (6) */
  
  /* repeat (2) to (6) for channel 3 */
  DMA1_Channel3->CPAR = (uint32_t) (&(TIM1->CCR2)); /* (2) */
  DMA1_Channel3->CMAR = (uint32_t)(&DutyCycle); /* (3) */
  DMA1_Channel3->CNDTR = 1; /* (4) */
  DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 \
                      | DMA_CCR_TEIE | DMA_CCR_CIRC; /* (5) */   
  DMA1_Channel3->CCR |= DMA_CCR_EN; /* (6) */

  /* Configure NVIC for DMA */
  /* (7) Enable Interrupt on DMA Channels x */
  /* (8) Set priority for DMA Channels x */  
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn); /* (7) */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn,3); /* (8) */
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
    if ((error == 0) && (Period != 0) && (DutyCycle <= Period) )
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
      error &= ~ERROR_DMA_XFER; /* Reset error on DMA transfer */
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
  * @brief  This function handles DMA Channel2 and 3 interrupt request.
  *         It only manages DMA error on channel 3
  * @param  None
  * @retval None
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  
  if ((DMA1->ISR & DMA_ISR_TEIF1) != 0) /* Test if transfer error on DMA channel 1 */
  {
    error |= ERROR_DMA_XFER; /* Report an error on DMA transfer */
    DMA1->IFCR |= DMA_IFCR_CTEIF1; /* Clear the flag */
  }
  else
  {
    error |= ERROR_UNEXPECTED_DMA_IT; /* Report unexpected DMA interrupt occurrence */
  }
}


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
