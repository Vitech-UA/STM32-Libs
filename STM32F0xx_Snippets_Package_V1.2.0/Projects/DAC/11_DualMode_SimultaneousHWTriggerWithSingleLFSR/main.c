/**
  ******************************************************************************
  * @file    11_DualMode_SimultaneousHWTriggerWithSingleLFSR/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the DAC in dual mode 
  *          in order to generate two simultaneous signal, the two channels 
  *          triggered by the same timer. Each channel having its own pseudonoise  
  *          generated with the signal with the same amplitude.
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO PA4, PA5
   - DAC
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
    - The code performs the DAC configuration and provides data in order 
      to generate a fixed signal on the DAC outputs PA4 and PA5.
    - The DAC is configured to add the same amplitude pseudonoise thru the LFSR on 
      each channel.
    - The Timer 7 is configured to generate an external trigger
      on TRGO each 2.5us.  
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


/* following defines are the DAC output value for each channel*/
#define DAC_OUT1_VALUE 3000
#define DAC_OUT2_VALUE 2000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIOasAnalog(void);
void ConfigureDAC(void);
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
  ConfigureTIM7();
  __WFI(); /* the wait mode should never be exited  as not interrupt enabled*/
  while(1);
}


/**
  * @brief  This function enables the peripheral clocks on GPIO port A
  *         and configures PA4 in Analog mode.
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
  *         and configures the DAC to be ready to generate a signal on DAC1_OUT
  *         and on DAC2_OUT synchronized by TIM7 HW trigger.
  *         It configures the two DAC channel WAVEx[1:0] bits as “01” 
  *         and the same LFSR mask value in the MAMPx[3:0] bits = 1000 for a 511-bits amplitude 
  *         The output buffer is enabled on each channel.
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureDAC(void)
{
  /* (1) Enable the peripheral clock of the DAC */
  /* (2) Configure WAVEx at 01 and LFSR mask amplitude (MAMPx) at 1000 for a 511-bits amplitude, 
         enable the DAC ch1 and ch2,
         disable buffer on ch1 and ch2,  
         select TIM7 as trigger by writing 010 in TSEL1 and TSEL2 */
  RCC->APB1ENR |= RCC_APB1ENR_DACEN; /* (1) */
  DAC->CR |= DAC_CR_WAVE1_0 | DAC_CR_WAVE2_0 | DAC_CR_MAMP1_3 | DAC_CR_MAMP2_3 \
           | DAC_CR_TSEL2_1 | DAC_CR_BOFF2 | DAC_CR_TEN2 | DAC_CR_EN2 \
           | DAC_CR_TSEL1_1 | DAC_CR_BOFF1 | DAC_CR_TEN1 | DAC_CR_EN1; /* (2) */  
  
  DAC->DHR12RD = (uint32_t)((DAC_OUT2_VALUE << 16) + DAC_OUT1_VALUE); /* Initialize the dual register */
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
