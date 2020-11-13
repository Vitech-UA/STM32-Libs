/**
  ******************************************************************************
  * @file    01_ReadDeviceId/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the GPIOs and TSC
  *          in order to perform acquisition and lit LEDs if touch detected.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO  PA0, PC8, PC9,
   - DBG
   - EXTI

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
   - Launch the program
   - Press the user button
   - ...
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
#define MCU_ID (0x448)
#define MCU_ID_MASK (0x00000EFF)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t MCU_Id = 0;

/* Private function prototypes -----------------------------------------------*/
void Configure_GPIO_LED(void);
void CheckId(void);

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

  Configure_GPIO_LED();
  CheckId();

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the Green LED pin on GPIO PC9
             - Configures the Orange LED pin on GPIO PC8
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_LED(void)
{
  /* Enable the peripheral clock of GPIOC */
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
  /* Select output mode (01) on PC8 and PC9 */
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9)) \
                 | (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
}

/**
  * @brief  This function check the MCU Id.
  * @param  None
  * @retval None
  */
__INLINE void CheckId(void)
{
  MCU_Id = DBGMCU->IDCODE; /* Read MCU Id, 32-bit access */
  
  if((MCU_Id & MCU_ID_MASK) == MCU_ID) /* Check Id */
  {
    GPIOC->BSRR = GPIO_BSRR_BS_9; /* lit green LED */
  }
  else
  {
    GPIOC->BSRR = GPIO_BSRR_BS_8; /* lit orange LED */
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
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
