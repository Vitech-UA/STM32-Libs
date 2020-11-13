/**
  ******************************************************************************
  * @file    20_DMA_BurstFeature/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the timer 
  *          to generate a PWM center-aligned signal and to modify 
  *          the duty cycle on each Update Event. 
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
  - RCC
  - TIMx
  - GPIO PB3, PB10, PB11 for TIM2_CH2, TIM2_CH3 and TIM2_CH4
  - DMA
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
    - This example configures the TIM2 in order to generate a PWM center-aligned 
      on OC2, OC3 and OC4, with a period of 16 microseconds and a variable duty 
      cycle.
      The GPIO PB3, PB10 and PB11, corresponding to TIM2_CH2/CH3/CH4, are 
      configured as alternate function and the AF2 is selected for all of them.
    - To test this example, the user must monitor the signal on PB3, PB10 
      and PB11.
    - This example can be easily ported on any other timer by modifying TIMx 
      definition. The corresponding GPIO must be also adapted according to 
      the datasheet.

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


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t Duty_Cycle_Table[3*10] = { 19, 29, 39, 69, 79, 89,
                                   119,129,139,169,179,189,
                                   219,229,239,269,279,289,
                                   319,329,339,359,379,389,
                                   419,429,439,469,479,489};
/* Private function prototypes -----------------------------------------------*/
void ConfigureTIMxAsPWM_CenterAligned(void);
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
  ConfigureTIMxAsPWM_CenterAligned();
  while (1)  
  {  
    __WFI();
  }        
}

/**
  * @brief  This function configures the TIMx as PWM mode 1 and center-aligned
  *         and enables the peripheral clock on TIMx and on GPIOA.
  *         It configures GPIO PA8 as Alternate function for TIM1_CH1
  *         To use another timer, channel or GPIO, the RCC and GPIO configuration 
  *         must be adapted according to the datasheet.
  *         In case of other timer, the interrupt sub-routine must also be renamed
  *         with the right handler and the NVIC configured correctly.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIMxAsPWM_CenterAligned(void)
{
  /* (1) Enable the peripheral clocks of Timer x and DMA*/
  /* (2) Enable the peripheral clock of GPIOB */
  /* (3) Select alternate function mode on GPIOB pin 3, 10 and 11 */
  /* (4) Select AF2 on PB3 in AFRL for TIM2_CH2 */
  /* (5) Select AF2 on PB10 and 11 in AFRH for TIM2_CH3 and TIM2_CH4 */
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* (1) */
  RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* (1) */
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN; /* (2) */  
  GPIOB->MODER = (GPIOB->MODER 
              & ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER10 | GPIO_MODER_MODER11)) 
               | (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1); /* (3) */
  GPIOB->AFR[0] |= 0x02 << (3 * 4); /* (4) */
  GPIOB->AFR[1] |= (0x02 << ((10-8) * 4)) | (0x02 << ((11-8) * 4)); /* (5) */
  
  /* (1) Set prescaler to 47, so APBCLK/48 i.e 1MHz */ 
  /* (2) Set ARR = 499, as timer clock is 1MHz and center-aligned counting,
         the period is 1000 us */
  /* (3) Set CCR2/3/4 = 49, the signal will be high during 14 us */
  /* (4) Select PWM mode 1 on OC2/3/4  (OCxM = 110),
         enable preload register on OC2/3/4 (OCxPE = 1, reset value) */
  /* (5) Select active high polarity on OC2/3/4 (CCxP = 0, reset value),
         enable the output on OC2/3/4 (CCxE = 1)*/
  /* (6) Enable output (MOE = 1)*/
  /* (7) Select center-aligned mode 1 (CMS = 01) */  
  /* (8) Force update generation (UG = 1) */
  
  TIMx->PSC = 47; /* (1) */
  TIMx->ARR = 499; /* (2) */
  TIMx->CCR2 = 20; /* (3) */
  TIMx->CCR3 = 30; /* (3) */
  TIMx->CCR4 = 40; /* (3) */  
  TIMx->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE; /* (4) */
  TIMx->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE
               | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE; /* (4) */
  TIMx->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; /* (5) */
  TIMx->BDTR |= TIM_BDTR_MOE; /* (6) */
  TIMx->CR1 |= TIM_CR1_CMS_0; /* (7) */
  TIMx->EGR |= TIM_EGR_UG; /* (8) */
  
  /* Configure DMA Burst Feature */
  /* Configure the corresponding DMA channel */ 
  /* (1) Set DMA channel peripheral address is the DMAR register address */
  /* (2) Set DMA channel memory address is the address of the buffer in the RAM 
         containing the data to be transferred by DMA into CCRx registers */
  /* (3) Set the number of data transfer to sizeof(Duty_Cycle_Table) */
  /* (4) Configure DMA transfer in CCR register
         enable the circular mode by setting CIRC bit (optional)
         set memory size to 16_bits MSIZE = 01 
         set peripheral size to 32_bits PSIZE = 10
         enable memory increment mode by setting MINC
         set data transfer direction read from memory by setting DIR  */
  /* (5) Configure TIMx_DCR register with DBL = 3 transfers 
         and DBA = (@TIMx->CCR2 - @TIMx->CR1) >> 2 = 0xE */
  /* (6) Enable the TIMx update DMA request by setting UDE bit in DIER register */
  /* (7) Enable TIMx */
  /* (8) Enable DMA channel */
  DMA1_Channel2->CPAR = (uint32_t)(&(TIMx->DMAR)); /* (1) */
  DMA1_Channel2->CMAR = (uint32_t)(Duty_Cycle_Table); /* (2) */
  DMA1_Channel2->CNDTR = 10*3; /* (3) */
  DMA1_Channel2->CCR |= DMA_CCR_CIRC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1
                      | DMA_CCR_MINC | DMA_CCR_DIR; /* (4) */
  TIMx->DCR = (3 << 8) 
            + ((((uint32_t)(&TIM2->CCR2)) - ((uint32_t)(&TIM2->CR1))) >> 2) ; /* (5) */
  TIMx->DIER |= TIM_DIER_UDE; /* (6) */
  TIMx->CR1 |= TIM_CR1_CEN; /* (7) */
  DMA1_Channel2->CCR |= DMA_CCR_EN; /* (8) */
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
