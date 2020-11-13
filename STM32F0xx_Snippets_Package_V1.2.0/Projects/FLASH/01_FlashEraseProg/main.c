/**
  ******************************************************************************
  * @file    01_FlashEraseAndProg/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to erase and program the Flash memory
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - Flash memory
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
    - This example erases the target page and program the first 32-bits word
      with a defined value.
      The programming is performed in two operations first the 16 less 
      significant bits (LSbits) on the first address then the 16 MSbits 
      on the next address.
      This test is done twice in case the erase value (OxFFFF) was already 
      programmed at the first address of the page before the erasing sequence.
      If this example is successful, the green led blinks regularly. 
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
#define FLASH_PAGE_SIZE         ((uint32_t)0x00000400)   /* FLASH Page Size */
#define FLASH_USER_START_ADDR   ((uint32_t)0x08002000)   /* Start @ of user Flash area */
#define DATA_TO_PROG            ((uint32_t)0xAA55CC33)   /* 32-bits value to be programmed */

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
volatile uint16_t error = 0;  //initialized at 0 and modified by the functions 
uint32_t test_to_be_performed_twice = 1; //this variable is set to 2 if the first address of the page to erase is yet erased

/* Private function prototypes -----------------------------------------------*/
void  ConfigureGPIO(void);
void  FlashOperationPreparation(void);
void  FlashErase(uint32_t page_addr);
void CheckFlashErase(uint32_t first_page_addr);
void FlashWord16Prog(uint32_t flash_addr, uint16_t data);
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
  FlashOperationPreparation();
  /* Check if the first address of the page is yet erased,this is only for this example */
  if (*(uint32_t *)(FLASH_USER_START_ADDR) == (uint32_t)0xFFFFFFFF) 
  {
    test_to_be_performed_twice = 2;
  }
  
  while (test_to_be_performed_twice-- > 0)
  {
    FlashErase(FLASH_USER_START_ADDR);
    CheckFlashErase(FLASH_USER_START_ADDR);
    FlashWord16Prog(FLASH_USER_START_ADDR, (uint16_t)DATA_TO_PROG);
    FlashWord16Prog(FLASH_USER_START_ADDR + 2, (uint16_t)(DATA_TO_PROG >> 16));
    
    /* Check the programming of the address */
    if  ((*(uint32_t *)(FLASH_USER_START_ADDR)) != DATA_TO_PROG)
    {
      error |= ERROR_PROG;
    }
  }
  
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
  while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (1) */  
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
  * @brief  This function erases a page of flash.
  *         The Page Erase bit (PER) is set at the beginning and reset at the end
  *         of the function, in case of successive erase, these two operations
  *         could be performed outside the function.
  * @param  page_addr is an address inside the page to erase
  * @retval None
  */
__INLINE void FlashErase(uint32_t page_addr)
{   
  /* (1) Set the PER bit in the FLASH_CR register to enable page erasing */
  /* (2) Program the FLASH_AR register to select a page to erase */
  /* (3) Set the STRT bit in the FLASH_CR register to start the erasing */
  /* (4) Wait until the BSY bit is reset in the FLASH_SR register */
  /* (5) Check the EOP flag in the FLASH_SR register */
  /* (6) Clear EOP flag by software by writing EOP at 1 */
  /* (7) Reset the PER Bit to disable the page erase */
  FLASH->CR |= FLASH_CR_PER; /* (1) */    
  FLASH->AR =  page_addr; /* (2) */    
  FLASH->CR |= FLASH_CR_STRT; /* (3) */    
  while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (4) */ 
  {
    /* For robust implementation, add here time-out management */
  }  
  if ((FLASH->SR & FLASH_SR_EOP) != 0)  /* (5) */
  {  
    FLASH->SR |= FLASH_SR_EOP; /* (6)*/
  }    
  /* Manage the error cases */
  else if ((FLASH->SR & FLASH_SR_WRPERR) != 0) /* Check Write protection error */
  {
    error |= ERROR_WRITE_PROTECTION; /* Report the error to the main progran */
    FLASH->SR |= FLASH_SR_WRPERR; /* Clear the flag by software by writing it at 1*/
  }
  else
  {
    error |= ERROR_UNKNOWN; /* Report the error to the main progran */
  }
  FLASH->CR &= ~FLASH_CR_PER; /* (7) */
}


/**
  * @brief  This function checks that the whole page has been correctly erased
  *         A word is erased while all its bits are set.
  * @param  first_page_addr is the first address of the page to erase
  * @retval None
  */
__INLINE void CheckFlashErase(uint32_t first_page_addr)
{
uint32_t i;  

  for (i=FLASH_PAGE_SIZE; i > 0;i-=4) /* Check the erasing of the page by reading all the page value */
  {
    if (*(uint32_t *)(first_page_addr + i -4) != (uint32_t)0xFFFFFFFF) /* compare with erased value, all bits at1 */
    {
      error |= ERROR_ERASE; /* report the error to the main progran */
    }
  }
}


/**
  * @brief  This function programs a 16-bit word.
  *         The Programming bit (PG) is set at the beginning and reset at the end
  *         of the function, in case of successive programming, these two operations
  *         could be performed outside the function.
  *         This function waits the end of programming, clears the appropriate bit in 
  *         the Status register and eventually reports an error. 
  * @param  flash_addr is the address to be programmed
  *         data is the 16-bit word to program
  * @retval None
  */
__INLINE void FlashWord16Prog(uint32_t flash_addr, uint16_t data)
{    
  /* (1) Set the PG bit in the FLASH_CR register to enable programming */
  /* (2) Perform the data write (half-word) at the desired address */
  /* (3) Wait until the BSY bit is reset in the FLASH_SR register */
  /* (4) Check the EOP flag in the FLASH_SR register */
  /* (5) clear it by software by writing it at 1 */
  /* (6) Reset the PG Bit to disable programming */
  FLASH->CR |= FLASH_CR_PG; /* (1) */
  *(__IO uint16_t*)(flash_addr) = data; /* (2) */
  while ((FLASH->SR & FLASH_SR_BSY) != 0) /* (3) */
  {
    /* For robust implementation, add here time-out management */
  }  
  if ((FLASH->SR & FLASH_SR_EOP) != 0)  /* (4) */
  {
    FLASH->SR |= FLASH_SR_EOP; /* (5) */
  }
  /* Manage the error cases */
  else if ((FLASH->SR & FLASH_SR_PGERR) != 0) /* Check Programming error */
  {      
    error = ERROR_PROG_FLAG;
    FLASH->SR |= FLASH_SR_PGERR; /* Clear it by software by writing EOP at 1*/
  }
  else if ((FLASH->SR & FLASH_SR_WRPERR) != 0) /* Check write protection */
  {      
    error = ERROR_WRITE_PROTECTION; 
    FLASH->SR |= FLASH_SR_WRPERR; /* Clear it by software by writing it at 1*/
  }
  else
  {
    error = ERROR_UNKNOWN; 
  }
  FLASH->CR &= ~FLASH_CR_PG; /* (6) */
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
