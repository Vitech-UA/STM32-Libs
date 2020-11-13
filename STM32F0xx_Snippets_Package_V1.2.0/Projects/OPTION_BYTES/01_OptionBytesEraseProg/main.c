/**
  ******************************************************************************
  * @file    01_OptionBytesEraseAndProg/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to program the Option bytes
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - Flash  memory
   - Option byte
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
    - This example erases the user data 0 and 1 option byte and program it
      with a defined value.
      This test is done twice in case the erase value (OxFFFF) was already programmed
      at the user data option byte before the erasing sequence.
      If this example is successful, the green led blinks regularly. 
      In case of failure, the orange led blinks many times according to the error
      then is off for a longer period before blinking again.
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
#define DATA_TO_PROG            ((uint16_t)0x1234)       /* 16-bits value to be programmed */
#define OPT0_READ                ((uint16_t)0xCB34)
#define OPT1_READ                ((uint16_t)0xED12)
/* Error codes used to make the orange led blinking */
#define ERROR_ERASE 0x01
#define ERROR_PROG  0x02
#define ERROR_PROG_FLAG 0x04
#define ERROR_WRITE_PROTECTION 0x08
#define ERROR_UNKNOWN 0x10


/* Delay value : short one is used for the error coding, long one in case of no error
   or between two bursts */
#define SHORT_DELAY 100
#define LONG_DELAY 1000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t error = 0; //initialized at 0 and modified by the functions 
uint32_t test_to_be_performed_twice = 1; //this variable is set to 2 if the option byte Data 0 is yet erased

/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void OptionFlashOperationPreparation(void);
void OptionErase(void);
void OptionProg(volatile uint16_t* opt_addr, uint8_t data);
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
  OptionFlashOperationPreparation();
  if (OB->DATA0 == (uint16_t)0xFFFF) /* Check if the option byte data 0 is yet erased, only for this example */
  {
    test_to_be_performed_twice = 2;
  }
  while (test_to_be_performed_twice-- > 0)
  {
    OptionErase();
    if (OB->DATA0 != (uint16_t)0xFFFF) /* Check the erasing of user data option byte*/
    {
      error |= ERROR_ERASE;
    }
    OptionProg(&(OB->RDP), (uint8_t)0xAA); /* Program the read protection */
    OptionProg(&(OB->DATA0), (uint8_t)DATA_TO_PROG); /* Program data 0 option byte */
    OptionProg(&(OB->DATA1), (uint8_t)(DATA_TO_PROG >> 8)); /* Program data 0 option byte */
    if  ((OB->RDP != 0x55AA) ||(OB->DATA0 != OPT0_READ) || (OB->DATA1 != OPT1_READ)) /* Check the programming of the address */
    {
      error |= ERROR_PROG;
    }
  }
  SysTick_Config(48000); /* 1ms config */
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
__INLINE void ConfigureGPIO(void)
{  
  /* (1) Enable the peripheral clock of GPIOC */
  /* (2) Select output mode (01) on GPIOC pin 8 and 9 */
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; /* (1) */  
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8|GPIO_MODER_MODER9)) \
               | (GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0); /* (2) */  
}


/**
  * @brief  This function prepares the option byte to be erased or programmed.
  *         It first checks no flash operation is on going,
  *         then unlocks both the flash and the option byte if it is locked.
  *         The flash must be unlocked to unlock the option bytes.
  * @param  None
  * @retval None
  */
__INLINE void OptionFlashOperationPreparation(void)
{  
  /* (1) Wait till no operation is on going */
  /* (2) Check that the Flash is unlocked */
  /* (3) Perform unlock sequence for Flash */
  /* (4) Check that the Option Bytes are unlocked */
  /* (5) Perform unlock sequence for Option Bytes */
  while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */  
  {
    /* For robust implementation, add here time-out management */
  }  
  if ((FLASH->CR & FLASH_CR_LOCK) != 0) /* (2) */
  {    
    FLASH->KEYR = FLASH_FKEY1; /* (3) */
    FLASH->KEYR = FLASH_FKEY2;
  }
  if ((FLASH->CR & FLASH_CR_OPTWRE) == 0) /* (4) */
  {
    FLASH->OPTKEYR = FLASH_OPTKEY1; /* (5) */
    FLASH->OPTKEYR = FLASH_OPTKEY2;
  }
}


/**
  * @brief  This function erases all the option bytes.
  * @param  None
  * @retval None
  */
__INLINE void OptionErase(void)
{
  /* (1) Set the OPTER bit in the FLASH_CR register to enable option byte erasing */
  /* (2) Set the STRT bit in the FLASH_CR register to start the erasing */
  /* (3) Wait until the BSY bit is reset in the FLASH_SR register */
  /* (4) Check the EOP flag in the FLASH_SR register */
  /* (5) Clear EOP flag by software by writing EOP at 1 */
  /* (6) Reset the PER Bit to disable the page erase */
  FLASH->CR |= FLASH_CR_OPTER; /* (1) */
  FLASH->CR |= FLASH_CR_STRT; /* (2) */    
  while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (3) */
  {
    /* For robust implementation, add here time-out management */
  }
  if ((FLASH->SR & FLASH_SR_EOP) != 0)  /* (4) */
  {
    FLASH->SR |= FLASH_SR_EOP; /* (5)*/
  }    
  /* Manage the error cases */
  else if ((FLASH->SR & FLASH_SR_WRPERR) != 0) /* Check Write protection error error */
  {
    error |= ERROR_WRITE_PROTECTION; /* Report error */
    FLASH->SR |= FLASH_SR_WRPERR;    /* Clear the flag */
  }
  else
  {
    error = ERROR_UNKNOWN; /* Report error */
  }
  FLASH->CR &= ~FLASH_CR_OPTER; /* (6) */
}


/**
  * @brief  This function programs an option byte.
  *         The Programming bit (OPTPG) is set at the beginning and reset at the end
  *         of the function, in case of successive programming, these two operations
  *         could be performed outside the function.
  *         This function waits the end of programming, clears the appropriate bit in 
  *         the Status register and eventually reports an error. 
  * @param  opt_addr is the address to be programmed
  *         data is the byte to program
  * @retval None
  */
__INLINE void OptionProg(volatile uint16_t* opt_addr, uint8_t data)
{    
  /* (1) Set the PG bit in the FLASH_CR register to enable programming */
  /* (2) Perform the data write */
  /* (3) Wait until the BSY bit is reset in the FLASH_SR register */
  /* (4) Check the EOP flag in the FLASH_SR register */
  /* (5) Clear the EOP flag by software by writing it at 1 */
  /* (6) Reset the PG Bit to disable programming */
  FLASH->CR |= FLASH_CR_OPTPG; /* (1) */
  *opt_addr = data; /* (2) */
  while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (3) */
  {
    /* For robust implementation, add here time-out management */
  }  
  if ((FLASH->SR & FLASH_SR_EOP) != 0)  /* (4) */
  {
    FLASH->SR |= FLASH_SR_EOP; /* (5) */
  }
  /* Manage the error cases */
  else if ((FLASH->SR & FLASH_SR_PGERR) != 0) /* Check programming error */
  {
    error = ERROR_PROG_FLAG; /* Report programming error */
    FLASH->SR |= FLASH_SR_PGERR; /* Clear the flag */
  }
  else if ((FLASH->SR & FLASH_SR_WRPERR) != 0)
  {
    error = ERROR_WRITE_PROTECTION; /* Report Write protection error */
    FLASH->SR |= FLASH_SR_WRPERR; /* Clear the flag */
  }     
  FLASH->CR &= ~FLASH_CR_OPTPG; /* (6) */
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
      long_counter = LONG_DELAY << 2;
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
