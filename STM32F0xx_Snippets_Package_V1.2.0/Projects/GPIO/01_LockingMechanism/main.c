/**
  ******************************************************************************
  * @file    01_LockingMechanism/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to lock the configuration of some pins 
   *         in a GPIO port.  
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - GPIO PA0, PA8 and PA9
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
    - The code configure port A and lock its configuration, then it attempts
      to write in the locked registers. 
    - In case the write is successful, the orange LED blinks meaning the lock has 
      failed, else the green led blinks slowly.
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

/* Error codes used to make the orange led blinking */
#define ERROR_LOCK_FAILED 0x01
#define ERROR_WRITING_OK 0x02
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t error = 0;  //initialized at 0 and modified by the functions 
/* Private function prototypes -----------------------------------------------*/
void  ConfigureGPIO(void);
void  LockGPIOA(uint16_t lock);
void  ChangeGPIOA_Configuration(void);
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
  LockGPIOA(1<<0 | 1<<8 | 1<<9); /* Locks PA0, PA8 and PA9 */
  ChangeGPIOA_Configuration();
  while (1) /* Infinite loop */
  {
  }
}

/**
  * @brief  This function enables the peripheral clocks on GPIO port A and port C,
  *         configures the green LED pin on GPIO PC9,
  *         configures the orange LED pin on GPIO PC8,
  *         keep default configuration for PA0 except select pull-down in PUPDR.
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureGPIO(void)
{  
  /* (1) Enable the peripheral clock of GPIOA and GPIOC */
  /* (2) Select output mode (01) on GPIOC pin 8 and 9 */
  /* (3) Select Pull-down (10) for PA0 */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN; /* (1) */  
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8|GPIO_MODER_MODER9)) \
               | (GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0); /* (2) */  
  GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPDR0)); /* (3) */
}


/**
  * @brief  This function locks the targeted pins of Port A configuration
            This function can be easily modified to lock Port B
  * @param  lock contains the port pin mask to be locked
  * @retval None
  */
void  LockGPIOA(uint16_t lock)
{
  /* (1) Write LCKK bit to 1 and set the pin bits to lock */
  /* (2) Write LCKK bit to 0 and set the pin bits to lock */
  /* (3) Write LCKK bit to 1 and set the pin bits to lock */
  /* (4) Read the Lock register */
  /* (5) Check the Lock register (optionnal) */
  GPIOA->LCKR = GPIO_LCKR_LCKK + lock; /* (1) */
  GPIOA->LCKR = lock; /* (2) */
  GPIOA->LCKR = GPIO_LCKR_LCKK + lock; /* (3) */
  GPIOA->LCKR; /* (4) */
  if ((GPIOA->LCKR & GPIO_LCKR_LCKK) == 0) /* (5) */
  {
    error |= ERROR_LOCK_FAILED; /* Report an error */
  }
}


/**
  * @brief  This function writes in the protecting registers
  *         and check they are not modified
  * @param  None
  * @retval None
  */
__INLINE void  ChangeGPIOA_Configuration(void)
{
uint32_t mode, pupd, ospeed, afr;

  /* Store the current configuration */
  mode = GPIOA->MODER;
  pupd = GPIOA->PUPDR;
  ospeed = GPIOA->OSPEEDR;
  afr = GPIOA->AFR[1];
  
  /* (1) Select output mode on GPIOA pin 8 and 9 */ 
  /* (2) Select Pull-up for PA0 */
  /* (3) Select high speed for GPIOA pin 8 (max 50MHz) - OSPEEDR8 = 11*/
  /* (4) Select AFR2 for PA8 and AFR3 for PA9 */
  GPIOA->MODER |= (GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0); /* (1) */  
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0; /* (2) */
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8; /* (3) */
  GPIOA->AFR[1] |= 2 + (3 << ((9-8) *4)); /* (4) */
  
  /* Check each configuration register has not been modified */
  if ((GPIOA->MODER != mode) || (GPIOA->PUPDR != pupd) || (GPIOA->OSPEEDR != ospeed) || (GPIOA->AFR[1] != afr))
  {
    error |= ERROR_WRITING_OK; /* Report an error */
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
