/**
  ******************************************************************************
  * @file    02_MassErase/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to mass erase the Flash memory
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - Flash  memory
   - GPIO PC8 and PC9
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
    - This example performs a mass erase.
      If this example is successful, the green led is light on. 
      In case of failure, the orange led blinks many times according to the error
      then is off for a longer period.
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
#define FLASH_MAIN_ADDR   ((uint32_t)0x08000000)   /* Start @ of user Flash area */

/* Error codes used to make the orange led blinking */
#define ERROR_ERASE (0x01)
#define ERROR_WRITE_PROTECTION (0x02)

/* Delay value : short one is used for the error coding, long one in case of no error
   or between two bursts */
#define SHORT_DELAY 200
#define LONG_DELAY 1000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t error = 0; /* Contain the error code, 0 while no error */
uint32_t test_to_be_performed_twice = 1; /* this variable is set to 2 if the first address 
                                            of the page to erase is yet at erased value */

/* Private function prototypes -----------------------------------------------*/
void  ConfigureGPIO(void);
void  FlashOperationPreparation(void);
void  FlashMassErase(void);
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
  GPIOC->BSRR = (uint16_t)(1 << 9); /* Light on green led on PC9 */
  FlashOperationPreparation();
  FlashMassErase();

  if (*(uint32_t *)(FLASH_MAIN_ADDR ) != (uint32_t)0xFFFFFFFF) /* Check the erasing of the page by reading all the page value */
  {
    error |= ERROR_ERASE; /* Report the error */
  }
    
  while (1) /* Infinite loop */
  {
  }
}


/**
  * @brief  This function enables the peripheral clocks on GPIO ports C,
  *         configures the Green LED pin on GPIO PC9,
  *         configures the orange LED pin on GPIO PC8,
  * @param  None
  * @retval None
  */
__INLINE void ConfigureGPIO(void)
{  
  /* (1) Enable the peripheral clock of GPIOC */
  /* (2) Select output mode (01) on GPIOC pin 8 and 9 */
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; /* (1) */  
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8|GPIO_MODER_MODER9)) \
               | (GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0); /* (2) */  
}


/**
  * @brief  This function prepares the flash to be erased or programmed.
  *         It first checks no flash operation is on going,
  *         then unlocks the flash if it is locked.
  * @param  None
  * @retval None
  */
__INLINE void FlashOperationPreparation(void)
{  
  /* (1) Wait till no operation is on going */
  /* (2) Check that the Flash is unlocked */
  /* (3) Perform unlock sequence */
  while ((FLASH->SR & FLASH_SR_BSY) != 0)  /* (1) */
  {
    /* For robust implementation, add here time-out management */
  }
  if ((FLASH->CR & FLASH_CR_LOCK) != 0) /* (2) */
  {    
    FLASH->KEYR = FLASH_FKEY1; /* (3) */
    FLASH->KEYR = FLASH_FKEY2;
  }
}


/**
  * @brief  This function performs a mass erase of the flash.
  *         This function can be loaded in RAM.
  * @param  None
  * @retval While successful, the function never returns except if executed from RAM
  */
__INLINE void FlashMassErase(void)
{   
  /* (1) Set the MER bit in the FLASH_CR register to enable mass erasing */
  /* (2) Set the STRT bit in the FLASH_CR register to start the erasing */
  /* (3) Wait until the BSY bit is reset in the FLASH_SR register */
  /* (4) Check the EOP flag in the FLASH_SR register */
  /* (5) Clear EOP flag by software by writing EOP at 1 */
  /* (6) Reset the PER Bit to disable the mass erase */
  FLASH->CR |= FLASH_CR_MER; /* (1) */
  FLASH->CR |= FLASH_CR_STRT; /* (2) */
  while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (3) */  
  {
    /* For robust implementation, add here time-out management */
  }  
  /*----------------------------------------------------------------------------
  WHILE THE MASS ERASE IS SUCCESSFULL 
  THE FOLLOWING INSTRUCTIONS ARE NEVER EXECUTED 
  EXCEPT IF EXECUTED FROM RAM
  ----------------------------------------------------------------------------*/
  GPIOC->BSRR = (1<<25); /* Light off green led on PC9   */
  if ((FLASH->SR & FLASH_SR_EOP) != 0) /* (4)*/
  {
    FLASH->SR |= FLASH_SR_EOP; /* (5) */
  }
  /* Manage the error cases */
  else if ((FLASH->SR & FLASH_SR_WRPERR) != 0)  /* Check Write protection error error */
  {
    error |= ERROR_WRITE_PROTECTION; /* Report the error */
    FLASH->SR |= FLASH_SR_WRPERR; /* Clear the flag */
  }  
  FLASH->CR &= ~FLASH_CR_MER; /* (6) */
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
