/**
  ******************************************************************************
  * @file    04_SynchronousMode/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the GPIOs and USART1 in
  *          synchronous mode in order to send and receive bytes.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO  PA8(USART1_CK), PA9(USART1_TX),PA10(USART1_RX),PA0, PC8, PC9,
         	 PA5 (SPI1_SCK), PA6 (SPI1_MISO), PA7 (SPI1_MOSI)
   - USART1
   - SPI1 to test
   - EXTI
   
 ===============================================================================
                    ##### How to use this example #####
 ===============================================================================
   - this file must be inserted in a project containing the following files :
      o system_stm32f0xx.c, startup_stm32f072xb.s
      o stm32f0xx.h to get the register definitions
      o CMSIS files

 ===============================================================================
                    ##### How to test this example #####
 ===============================================================================
   - Solder SB29 and SB30
   - Connect PA8/PA5, PA9/PA7, PA10/PA6.
   - Launch the program
   - Press the user button
   - The green LEDs should toggle if transmission and reception are OK
     (Synchronous mode)
   - Else the orange LED lit

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
#define USART_DATA (0xCA)
#define SPI_DATA (0xDE)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Configure_GPIO_LED(void);
void Configure_GPIO_SPI1(void);
void Configure_SPI1(void);
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
  Configure_GPIO_SPI1();
  Configure_SPI1();
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
             - Configures the SPI1 pins on GPIO PA5 PA6 PA7
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_SPI1(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
  /* GPIO configuration for SPI1 signals */
  /* (1) Select AF mode (10) on PA5, PA6 and PA7 */
  /* (2) AF0 for SPI1 signals */
  GPIOA->MODER = (GPIOA->MODER 
                  & ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7))\
                | (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); /* (1) */
  GPIOA->AFR[0] = (GPIOA->AFR[0] & \
                   ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7)); /* (2) */
}

/**
  * @brief  This function configures SPI1.
  * @param  None
  * @retval None
  */
__INLINE void Configure_SPI1(void)
{
  /* Enable the peripheral clock SPI1 */
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  /* (1) Configure SPI1 in slave NSS soft */
  /*     SS at low, slave, CPOL and CPHA at zero (rising first edge) */
  /* (2) 8-bit transfer */
  SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SPE; /* (1) */
  SPI1->CR2 = SPI_CR2_FRXTH | SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2; /* (2) */
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the USART1 pins on GPIO PA8, PA9 and PA10
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_USART1(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
  /* GPIO configuration for USART1 signals */
  /* (1) Select AF mode (10) on PA8, PA9 and PA10 */
  /* (2) AF1 for USART1 signals */
  GPIOA->MODER = (GPIOA->MODER \
               & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9 | GPIO_MODER_MODER10))\
           | (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1)\
                 ; /* (1) */
  GPIOA->AFR[1] = (GPIOA->AFR[1] & \
                  ~ (GPIO_AFRH_AFRH0 | GPIO_AFRH_AFRH1 | GPIO_AFRH_AFRH2)) \
                  | (1 << (0 * 4)) | (1 << (1 * 4)) | (1 << (2 * 4)); /* (2) */
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
  /* (2) Synchronous mode */
  /*     CPOL and CPHA = 0 => rising first edge */
  /*     Last bit clock pulse */
  /*     Most significant bit first in transmit/receive */
  /* (3) 8 data bit, 1 start bit, 1 stop bit, no parity */
  /*     Transmission enabled, reception enabled */
  USART1->BRR = 480000 / 96; /* (1) */
  USART1->CR2 = USART_CR2_MSBFIRST | USART_CR2_CLKEN | USART_CR2_LBCL; /* (2) */
  USART1->CR1 = USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE; /* (3) */
  
  /* polling idle frame Transmission w/o clock */
  while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC)
  { 
    /* add time out here for a robust application */
  }
  USART1->ICR |= USART_ICR_TCCF;/* clear TC flag */
  USART1->CR1 |= USART_CR1_TCIE;/* enable TC interrupt */
  
  /* Configure IT */
  /* (4) Set priority for USART1_IRQn */
  /* (5) Enable USART1_IRQn */
  NVIC_SetPriority(USART1_IRQn, 0); /* (4) */
  NVIC_EnableIRQ(USART1_IRQn); /* (5) */
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
  
  /* start 8-bit synchronous transmission */
  *(uint8_t *)&(SPI1->DR) = SPI_DATA;/* to test USART reception */
  USART1->TDR = USART_DATA;/* will inititiate 8-bit transmission if TXE */
}

/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  uint8_t SPI_Data = 0;
  uint8_t USART_Data = 0;
	
  if((USART1->ISR & USART_ISR_TC) == USART_ISR_TC)
  {
    USART1->ICR |= USART_ICR_TCCF;/* clear transfer complete flag */
  }
  else if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
  {
    USART_Data = USART1->RDR;/* receive data, clear flag */
    SPI_Data = SPI1->DR;/* Get transmit data to SPI */
  
    if((USART_Data == SPI_DATA)&&(SPI_Data == USART_DATA))
    {
      GPIOC->ODR ^= GPIO_ODR_9 ;/* Toggle Green LED */
    }
    else
    {
      GPIOC->BSRR = GPIO_BSRR_BS_8;/* Lit Orange LED */
    }
  }
  else
  {
    NVIC_DisableIRQ(USART1_IRQn);/* Disable USART1_IRQn */
  }
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
