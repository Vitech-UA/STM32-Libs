/**
  ******************************************************************************
  * @file    08_DualMode_TriggerWithDfferentTriangle/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the DAC in dual mode 
  *          in order to generate two independent triangle waves, each channel triggered 
  *          by its own timer. Each channel having its own amplitude 
  *          and frequency.
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO PA4, PA5
   - DAC
   - TIM6
   - TIM7
   - WFI

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
    - The code performs the DAC configuration to generate a triangle wave signal 
      on the DAC outputs PA4 and PA5.
    - Each channel gets its own amplitude : 1023-bits for ch1 and 4095 for ch2
      and its own frequency. 
    - The Timer 6 is configured to generate an external trigger
      on TRGO each 1us : the period will be 1us x 2 x AMPLITUDE i.e. 2.048ms.
    - The Timer 7 is configured to generate an external trigger
      on TRGO each 2.5us so the period will 20.48ms 
    - The signal can be monitored with an oscilloscope on PA4 and on PA5.
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


/* following defines are the DAC output value (offset) for each channel*/
#define DAC_OUT1_VALUE 3000
#define DAC_OUT2_VALUE 0

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIOasAnalog(void);
void ConfigureDAC(void);
void ConfigureTIM6(void);
void ConfigureTIM7(void);
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
  ConfigureGPIOasAnalog();
  ConfigureDAC();
  ConfigureTIM6();
  ConfigureTIM7();
  __WFI(); /* the wait mode should never be exited as not interrupt enabled*/
  while(1);
}


/**
  * @brief  This function enables the peripheral clocks on GPIO port A
  *         and configures PA4 and PA5 in Analog mode.
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureGPIOasAnalog(void)
{
  /* (1) Enable the peripheral clock of GPIOA */
  /* (2) Select analog mode for PA4 and PA5 */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (1) */
  GPIOA->MODER |= GPIO_MODER_MODER4 | GPIO_MODER_MODER5; /* (2) */
}


/**
  * @brief  This function enables the peripheral clocks on DAC
  *         and configures the DAC to be to generate a triangle wave on DAC1_OUT
  *         synchronized by TIM6 HW trigger and on DAC2_OUT synchronized by 
  *         TIM7 HW trigger.
  *         It configures the two DAC channel WAVEx[1:0] bits as “10”, 
  *         Amplitude value in the MAMP1[3:0] bits = 1001 for a 1023-bits amplitude 
  *         Amplitude value in the MAMP2[3:0] bits = 1011 for a 4095-bits amplitude 
  *         The output buffer is enabled on each channel.
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureDAC(void)
{
  /* (1) Enable the peripheral clock of the DAC */
  /* (2) Configure WAVEx at 10,
         Configure mask amplitude for ch1 (MAMP1) at 1001 for a 1023-bits amplitude
         and mask amplitude for ch2 (MAMP1) at 1011 for a 4095-bits amplitude, 
         enable the DAC ch1 and ch2,
         disable buffer on ch1 and ch2,  
         select TIM7 as trigger by writing 010 in TSEL2
         and select TIM6 as trigger by keeping 000 in TSEL1 */
  RCC->APB1ENR |= RCC_APB1ENR_DACEN; /* (1) */
  DAC->CR |= DAC_CR_WAVE1_1 | DAC_CR_WAVE2_1 | DAC_CR_MAMP1_3 | DAC_CR_MAMP1_0 \
           | DAC_CR_MAMP2_3 | DAC_CR_MAMP2_1 | DAC_CR_MAMP2_0 \
           | DAC_CR_TSEL2_1 | DAC_CR_BOFF2 | DAC_CR_TEN2 | DAC_CR_EN2 \
           | DAC_CR_BOFF1 | DAC_CR_TEN1 | DAC_CR_EN1; /* (2) */  
  
  DAC->DHR12R1 = DAC_OUT1_VALUE; /* Define the low value of the triangle on channel1 */
  DAC->DHR12R2 = DAC_OUT2_VALUE; /* Define the low value of the triangle on channel2 */
}


/**
  * @brief  This function configures the Timer 6 to generate an external trigger
  *         on TRGO each microsecond.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIM6(void)
{
  /* (1) Enable the peripheral clock of the TIM6 */ 
  /* (2) Configure MMS=010 to output a rising edge at each update event */
  /* (3) Select PCLK/2 i.e. 48MHz/2=24MHz */
  /* (4) Set one update event each 1 microsecond */
  /* (5) Enable TIM6 */
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; /* (1) */ 
  TIM6->CR2 |= TIM_CR2_MMS_1; /* (2) */
  TIM6->PSC = 1; /* (3) */
  TIM6->ARR = (uint16_t)24; /* (4) */
  TIM6->CR1 |= TIM_CR1_CEN; /* (5) */
}



/**
  * @brief  This function configures the Timer 7 to generate an external trigger
  *         on TRGO each 2.5 microseconds.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIM7(void)
{
  /* (1) Enable the peripheral clock of the TIM7 */ 
  /* (2) Configure MMS=010 to output a rising edge at each update event */
  /* (3) Select PCLK/2 i.e. 48MHz/2=24MHz */
  /* (4) Set one update event each 2.5 microseconds */
  /* (5) Enable TIM7 */
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; /* (1) */ 
  TIM7->CR2 |= TIM_CR2_MMS_1; /* (2) */
  TIM7->PSC = 1; /* (3) */
  TIM7->ARR = (uint16_t)60; /* (4) */
  TIM7->CR1 |= TIM_CR1_CEN; /* (5) */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
