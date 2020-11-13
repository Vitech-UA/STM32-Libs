/**
  ******************************************************************************
  * @file    02_Receiver/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the GPIOs and USART1 
  *          in order to receive byte and deal with it.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO PA9(USART1_TX),PA10(USART1_RX),PA0,PC8,PC9
   - USART1
   - EXTI
   
 ===============================================================================
                    ##### How to use this example #####
 ===============================================================================
    - this file must be inserted in a project containing  the following files :
      o system_stm32f0xx.c, startup_stm32f072xb.s
      o stm32f0xx.h to get the register definitions
      o CMSIS files
    - Plug cable USB to TTL 3V3.

 ===============================================================================
                    ##### How to test this example #####
 ===============================================================================
   - Plug cable " USB to TTL 3V3 " (from FTDIChip), to PA9 and PA10.
   - Launch serial communication SW on PC
   - Launch the program
   - Send with PC "g" or "G" to toggle green LED on the board 
   - Send with PC "o" or "O" to toggle orange LED on the board 

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
void Configure_GPIO_LED(void);
void Configure_GPIO_USART1(void);
void Configure_USART1(void);
void Configure_GPIO_Button(void);
void Configure_EXTI(void);

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
  Configure_GPIO_USART1();
  Configure_USART1();
  Configure_GPIO_Button();
  Configure_EXTI();
  
  /* Reception done in USART1 IRQ handler */
	
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
  * @brief  This function :
             - Enables GPIO clock
             - Configures the USART1 pins on GPIO PB6 PB7
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_USART1(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
  /* GPIO configuration for USART1 signals */
  /* (1) Select AF mode (10) on PA9 and PA10 */
  /* (2) AF1 for USART1 signals */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9|GPIO_MODER_MODER10))\
                 | (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1); /* (1) */
  GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFRH1 | GPIO_AFRH_AFRH2))\
                  | (1 << (1 * 4)) | (1 << (2 * 4)); /* (2) */
}

/**
  * @brief  This function configures USART1.
  * @param  None
  * @retval None
  */
__INLINE void Configure_USART1(void)
{
  /* Enable the peripheral clock USART1 */
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  /* Configure USART1 */
  /* (1) oversampling by 16, 9600 baud */
  /* (2) 8 data bit, 1 start bit, 1 stop bit, no parity, reception mode */
  USART1->BRR = 480000 / 96; /* (1) */
  USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE; /* (2) */
  
  /* Configure IT */
  /* (3) Set priority for USART1_IRQn */
  /* (4) Enable USART1_IRQn */
  NVIC_SetPriority(USART1_IRQn, 0); /* (3) */
  NVIC_EnableIRQ(USART1_IRQn); /* (4) */
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the Push Button GPIO PA0
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_Button(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
  /* Select mode */
  /* Select input mode (00) on PA0 */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER0));
}

/**
  * @brief  This function configures EXTI.
  * @param  None
  * @retval None
  */
__INLINE void Configure_EXTI(void)
{
  /* Configure Syscfg, exti and nvic for pushbutton PA0 */
  /* (1) PA0 as source input */
  /* (2) unmask port 0 */
  /* (3) Rising edge */
  /* (4) Set priority */
  /* (5) Enable EXTI0_1_IRQn */
  SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0) | SYSCFG_EXTICR1_EXTI0_PA; /* (1) */ 
  EXTI->IMR |= EXTI_IMR_MR0; /* (2) */ 
  EXTI->RTSR |= EXTI_RTSR_TR0; /* (3) */ 
  NVIC_SetPriority(EXTI0_1_IRQn, 0); /* (4) */ 
  NVIC_EnableIRQ(EXTI0_1_IRQn); /* (5) */ 
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
  * @brief  This function handles EXTI 0 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_1_IRQHandler(void)
{
  EXTI->PR |= 1;	
}

/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  uint8_t chartoreceive = 0;
  
  if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
  {
    chartoreceive = (uint8_t)(USART1->RDR); /* Receive data, clear flag */
          
    switch(chartoreceive)
    {
    case 'g':
    case 'G': GPIOC->ODR ^= GPIO_ODR_9; /* toggle green LED */
              break;
    case 'o':
    case 'O': GPIOC->ODR ^= GPIO_ODR_8; /* toggle orange LED */
              break;
    default: break;
    }
  }
  else
  {
          NVIC_DisableIRQ(USART1_IRQn); /* Disable USART1_IRQn */
  }
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
