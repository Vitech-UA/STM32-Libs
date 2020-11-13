/**
  ******************************************************************************
  * @file    08_SmartCardMode/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the GPIOs and USART1 
  *          in order to send byte in smart card mode.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO PA8(USART1_CK),PA9(USART1_TX),PA0,PC8,PC9
   - USART1
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
   - A byte is transmit in smart card mode (clock on PA8, transmit data on PA9)
   - The green LED should toggle if everything goes well
     (transmit request)

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
#define BYTE_TO_TRANSMIT (0xDE)

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

  /* Start transmission in button IRQ handler */
	
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
  /* (1) Open drain for PA9 */
  /* (2) PU for PA9 */
  /* (3) Max speed for PA8 */
  /* (4) Select AF mode (10) on PA8 and PA9 */
  /* (5) AF1 for USART1 signals */
  GPIOA->OTYPER |= GPIO_OTYPER_OT_9; /* (1) */
  GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPDR9))\
                  | (GPIO_PUPDR_PUPDR9_0); /* (2) */
  GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(GPIO_OSPEEDR_OSPEEDR8))\
                  | (GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1); /* (3) */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9))\
                  | (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1); /* (4) */
  GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFRH0 | GPIO_AFRH_AFRH1))\
                  | (1 << (0 * 4)) | (1 << (1 * 4)); /* (5) */
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
  /* (2) Clock divided by 16 = 3MHz */
  /* (3) Samrt card mode enable */
  /* (4) 1.5 stop bits, clock enbale */
  /* (5) 8-data bit plus parity, 1 start bit */
  USART1->BRR = 480000 / 96; /* (1) */
  USART1->GTPR = 16 >> 1; /* (2) */
  USART1->CR3 = USART_CR3_SCEN; /* (3) */
  USART1->CR2 = USART_CR2_STOP_1 | USART_CR2_STOP_0 | USART_CR2_CLKEN; /* (4) */
  USART1->CR1 = USART_CR1_M | USART_CR1_PCE | USART_CR1_TE | USART_CR1_UE; /* (5) */
  
  /* Polling idle frame transmission transfer complete (this frame is not sent) */
  while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC)
  { 
    /* add time out here for a robust application */
  }
  USART1->ICR |= USART_ICR_TCCF;/* clear TC flag */
  USART1->CR1 |= USART_CR1_TCIE;/* enable TC interrupt */
  
  /* Configure IT */
  /* (6) Set priority for USART1_IRQn */
  /* (7) Enable USART1_IRQn */
  NVIC_SetPriority(USART1_IRQn, 0); /* (6) */
  NVIC_EnableIRQ(USART1_IRQn); /* (7) */
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
	
  /* start USART transmission */
  USART1->TDR = BYTE_TO_TRANSMIT; /* Will inititiate TC if TXE */
}

/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  if((USART1->ISR & USART_ISR_TC) == USART_ISR_TC)
  {
    USART1->ICR |= USART_ICR_TCCF; /* Clear transfer complete flag */
    GPIOC->ODR ^= GPIO_ODR_9 ; /* Toggle Green LED */
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
