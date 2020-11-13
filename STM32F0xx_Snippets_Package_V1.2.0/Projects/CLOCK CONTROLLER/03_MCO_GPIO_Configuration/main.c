/**
  ******************************************************************************
  * @file    03_MCO_GPIO_Configuration/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to output the system clock on MCO
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO PA8,PC8,9

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
    - This examples configure the PA8 to output the system clock. 
    - The signal on PA8 @48MHz can be monitored with an oscilloscope

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

/** @addtogroup PPP_Example_Output
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
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
  RCC->CFGR |= RCC_CFGR_MCO_SYSCLK; /* Select system clock to be output on the MCO without prescaler */
    
  /* Infinite loop */
  while (1)
  {
  }
}


/**
  * @brief  This function enables the peripheral clocks on GPIO ports A and C,
  *         configures the Green LED pin on GPIO PC9,
  *         configures the orange LED pin on GPIO PC8,
  *         and configures GPIO PA8 as Alternate function and output high speed.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureGPIO(void)
{
  /* (1) Enable the peripheral clock of GPIOC and GPIOA */
  /* (2) Select output mode (01) on GPIOC pin 8 and 9 */
  /* (3) Select alternate function mode on GPIOA pin 8 */
  /* (4) Select high speed for GPIOA pin 8 (max 50MHz) - OSPEEDR8 = 11 */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN; /* (1) */
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9)) \
               | (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0); /* (2) */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER8)) | (GPIO_MODER_MODER8_1); /* (3) */
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8; /* (4) */
  
  /* MCO is AFR0 on PA8 and Alternate function 0 is selected by default so no need to modify GPIOA_AFR */
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
