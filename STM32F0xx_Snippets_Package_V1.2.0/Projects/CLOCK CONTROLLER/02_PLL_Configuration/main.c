/**
  ******************************************************************************
  * @file    02_PLL_Configuration/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to change the PLL factor.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO PC8,9
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
    - When entering in main.c the PLL has been configured to x12 by SystemInit()
    - This example makes the green LED toggling cadenced by the system clock
    - This example modifies the PLL to x6, so the blinking is slower
    - Once the PLL is enabled, the orange LED is switched on.

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


/* Delay value */
#define LONG_DELAY 500

#define COUNTER_INIT 10

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t Counter = COUNTER_INIT; //the counter is only blinks the led during few second before changing the PLL
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void ChangePLL(void);
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
  SysTick_Config(48000); /* 1ms config */
  while (Counter != 0) // Wait for Counter reset by SysTick IRQ
  {
  }
  ChangePLL();         
  GPIOC->BSRR = (1<<8); //switch the orange LED on PC8

  /* Infinite loop */
  while (1)
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
  * @brief  This function switches the system clock to HSI
  * @param  None
  * @retval None
  */
__INLINE void ChangePLL(void)
{
  /* (1)  Test if PLL is used as System clock */
  /* (2)  Select HSI as system clock */
  /* (3)  Wait for HSI switched */
  /* (4)  Disable the PLL */
  /* (5)  Wait until PLLRDY is cleared */
  /* (6)  Set the PLL multiplier to 6 */
  /* (7)  Enable the PLL */ 
  /* (8)  Wait until PLLRDY is set */
  /* (9)  Select PLL as system clock */
  /* (10) Wait until the PLL is switched on */
  if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) /* (1) */
  {          
    RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW); /* (2) */
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) /* (3) */
    {
      /* For robust implementation, add here time-out management */      
    }
  }
  RCC->CR &= (uint32_t)(~RCC_CR_PLLON);/* (4) */        
  while((RCC->CR & RCC_CR_PLLRDY) != 0) /* (5) */
  {
    /* For robust implementation, add here time-out management */    
  }
  RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PLLMUL)) | (RCC_CFGR_PLLMUL6); /* (6) */
  RCC->CR |= RCC_CR_PLLON; /* (7) */
  while((RCC->CR & RCC_CR_PLLRDY) == 0) /* (8) */
  {
    /* For robust implementation, add here time-out management */
  }
  RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL); /* (9) */
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) /* (10) */
  {
    /* For robust implementation, add here time-out management */
  }
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
  
  if (long_counter-- == 0) 
  {
    /* the following instruction can only be used if no ISR modifies GPIOC ODR
       either by writing directly it or by using GPIOC BSRR or BRR 
       else a toggle mechanism must be implemented using GPIOC BSRR and/or BRR
    */
    GPIOC->ODR ^= (1<<9);//toggle green led on PC9
    long_counter = LONG_DELAY;
    if (Counter != 0)
    {
      Counter--; /* this variable is used to trigger the PLL change once */
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
