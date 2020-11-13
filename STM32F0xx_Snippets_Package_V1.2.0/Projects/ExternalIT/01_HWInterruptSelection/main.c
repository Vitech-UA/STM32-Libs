/**
  ******************************************************************************
  * @file    01_HWInterruptSelection/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure a GPIO to use it
  *          as external interrupt
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - PA0, PC9

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
    - push the User button, this toggles the green led

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
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void  ConfigureGPIO(void);
void ConfigureExternalIT(void);
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
  ConfigureExternalIT();
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
  * @brief  This function enables the peripheral clocks on GPIO port A,
  *         configures the EXTI register and NVIC IRQ.
  *         PA0 is kept in the default configuration (input,no pull-up, no pull-down)
  *         SYSCFG_EXTICR1 is kept at its reset value to select Port A for EXTI0 
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureExternalIT(void)
{  
  /* (1) Enable the peripheral clock of GPIOA */ 
  /* (2) Select Port A for pin 0 external interrupt by writing 0000 in EXTI0 (reset value)*/
  /* (3) Configure the corresponding mask bit in the EXTI_IMR register */
  /* (4) Configure the Trigger Selection bits of the Interrupt line on rising edge*/
  /* (5) Configure the Trigger Selection bits of the Interrupt line on falling edge*/
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (1) */
  //SYSCFG->EXTICR[1] &= (uint16_t)~SYSCFG_EXTICR1_EXTI0_PA; /* (2) */
  EXTI->IMR = 0x0001; /* (3) */ 
  EXTI->RTSR = 0x0001; /* (4) */
  EXTI->FTSR = 0x0001; /* (5) */
  
  /* Configure NVIC for External Interrupt */
  /* (6) Enable Interrupt on EXTI0_1 */
  /* (7) Set priority for EXTI0_1 */
  NVIC_EnableIRQ(EXTI0_1_IRQn); /* (6) */
  NVIC_SetPriority(EXTI0_1_IRQn,0); /* (7) */
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
  * @brief  This function handles EXTI0_1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_1_IRQHandler(void)
{
  if ((EXTI->PR & 0x0001) != 0)  /* Check line 0 has triggered the IT */
  {
    EXTI->PR |= 0x0001; /* Clear the pending bit */
    GPIOC->ODR ^= (1<<9); /* Toggle green led on PC9 */
  }
  else /* Should never occur */
  {
    GPIOC->BSRR = (1<<8); /* Switch on orange led to report an error */
  }
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
