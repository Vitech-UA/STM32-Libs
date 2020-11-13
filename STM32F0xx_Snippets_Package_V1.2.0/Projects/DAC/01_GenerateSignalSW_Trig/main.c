/**
  ******************************************************************************
  * @file    01_GenerateSignalSW_Trig/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the DAC in order to 
  *          generate a burst signal between the min and max value.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO PA4
   - DAC

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
      to generate a burst signal on the DAC output PA4.
    - The DAC amplitude is centered on the lower and higher DAC output 
      with buffer as stated in the electrical parameters of the datasheet.
    - The signal can be monitored with an oscilloscope on PA4.
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
/* INCREMENT is the burst step */
#define INCREMENT 5

/* following defines are voltages expressed in millivolts */
#define VDD_APPLI 3000
#define LOWER_DAC_OUT_VOLTAGE (uint32_t) (4096 * 200/VDD_APPLI)
#define HIGHER_DAC_OUT_VOLTAGE (uint32_t)(4095 - LOWER_DAC_OUT_VOLTAGE)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void  ConfigureGPIOasAnalog(void);
void  ConfigureDAC(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  uint16_t x;
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f072xb.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */
  ConfigureGPIOasAnalog();
  ConfigureDAC();
  while (1) /* Infinite loop */
  {
    for (x = LOWER_DAC_OUT_VOLTAGE; x < HIGHER_DAC_OUT_VOLTAGE; x+= INCREMENT)
    {
      DAC->DHR12R1 = x;
    }  
  }
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
  /* (2) Select analog mode for PA4 */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (1) */
  GPIOA->MODER |= GPIO_MODER_MODER4; /* (2) */
}


/**
  * @brief  This function enables the peripheral clocks on DAC
  *         and configures the DAC to be ready to generate signal on DAC1_OUT
  *         as soon as data is written in the DAC_DHR12R1 i.e. 12-bit left aligned data.
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureDAC(void)
{
  /* (1) Enable the peripheral clock of the DAC */
  /* (2) Enable the DAC ch1 */
  RCC->APB1ENR |= RCC_APB1ENR_DACEN; /* (1) */
  DAC->CR |=  DAC_CR_EN1; /* (2) */  
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
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
