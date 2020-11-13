/**
  ******************************************************************************
  * @file    01_HSE_Start/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to start the HSE 
  *          to use it as system clock.
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
    - On the discovery board ensure that SB19 is closed and SB17 is open.
    - The HSE on OSC_IN is provided by the MCO of the STM32F1 
      and is clocked at 8MHz.
    - This examples starts the HSE and use it as system clock
      The green LED blinks once HSE is started.

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


/* Delay value : short one is used for the error coding, long one in case of no error
   or between two bursts */
#define SHORT_DELAY 200
#define LONG_DELAY 1000

/* Error codes used to make the orange led blinking */
#define ERROR_CSS 0x01
#define ERROR_HSE_LOST 0x02
#define ERROR_UNEXPECTED_RCC_IRQ 0x04

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t error = 0xFF;  //initialized at 0xFF and modified by the functions 
/* Private function prototypes -----------------------------------------------*/
void  ConfigureGPIO(void);
void StartHSE(void);
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
  SysTick_Config(8000);/* 1ms config with HSE 8MHz*/
  StartHSE();
  while (1) /* Infinite loop */
  {
    if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE)
    {
      if (error == 0xFF)
      {
        error = 0;
      }
    }
    else
    {
      if (error == 0)
      {
        error = ERROR_HSE_LOST;
      }
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
  * @brief  This function enables the interrupton HSE ready,
  *         and start the HSE as external clock.
  * @param  None
  * @retval None
  */
__INLINE void StartHSE(void)
{
  /* Configure NVIC for RCC */
  /* (1) Enable Interrupt on RCC */
  /* (2) Set priority for RCC */
  NVIC_EnableIRQ(RCC_CRS_IRQn); /* (1)*/
  NVIC_SetPriority(RCC_CRS_IRQn,0); /* (2) */
  
  /* (1) Enable interrupt on HSE ready */
  /* (2) Enable the CSS 
         Enable the HSE and set HSEBYP to use the external clock 
         instead of an oscillator 
         Enable HSE */
  /* Note : the clock is switched to HSE in the RCC_CRS_IRQHandler ISR */
  RCC->CIR |= RCC_CIR_HSERDYIE; /* (1) */  
  RCC->CR |= RCC_CR_CSSON | RCC_CR_HSEBYP | RCC_CR_HSEON; /* (2) */  
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
  if ((RCC->CIR & RCC_CIR_CSSF) != 0)
  {
    error = ERROR_CSS; /* Report the error */
    RCC->CIR |= RCC_CIR_CSSC; /* Clear the flag */
  }
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
    if(error == 0)
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
  * @brief  This function handles RCC interrupt request 
  *         and switch the system clock to HSE.
  * @param  None
  * @retval None
  */
void RCC_CRS_IRQHandler(void)
{
  /* (1) Check the flag HSE ready */
  /* (2) Clear the flag HSE ready */
  /* (3) Switch the system clock to HSE */
  
  if ((RCC->CIR & RCC_CIR_HSERDYF) != 0) /* (1) */
  {
    RCC->CIR |= RCC_CIR_HSERDYC; /* (2) */    
    RCC->CFGR = ((RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_0); /* (3) */
  }
  else
  {
    error = ERROR_UNEXPECTED_RCC_IRQ; /* Report an error */
  }
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
