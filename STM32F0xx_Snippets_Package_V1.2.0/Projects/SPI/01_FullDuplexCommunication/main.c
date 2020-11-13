/**
  ******************************************************************************
  * @file    01_FullDuplexCommunication/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the GPIOs and SPI1(master)
  *          and SPI2 (slave) in full duplex communication in order to send and 
  *          receive bytes.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO  PA0, PC6, PC7, PC8, PC9,
           PA4(SPI1_NSS), PA5(SPI1_SCK), PA6(SPI1_MISO), PA7(SPI1_MOSI), 
           PB12(SPI2_NSS), PB13(SPI2_SCK), PB14(SPI2_MISO), PB15(SPI2_MOSI)
   - SPI1, SPI2
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
   - Solder SB29 and SB30 on the board
   - Connect PA4/PB12, PA5/PB13, PA6/PB14, PA7/PB15.
   - Launch the program
   - Press the button
   - The green and blue LEDs toggles (message transmit and receive)
   - Else the red and orange LEDs lit

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
#define SPI2_DATA (0xDE)
#define SPI1_DATA (0xCA)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Configure_GPIO_LED(void);
void Configure_GPIO_SPI1(void);
void Configure_SPI1(void);
void Configure_GPIO_SPI2(void);
void Configure_SPI2(void);
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
	GPIOC->BSRR = GPIO_BSRR_BS_7; /* lit blue LED */
  Configure_GPIO_SPI1();
  Configure_SPI1();
  Configure_GPIO_SPI2();
  Configure_SPI2();
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
             - Configures the orange LED pin on GPIO PC8
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_LED(void)
{
  /* Enable the peripheral clock of GPIOC */
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
  /* Select output mode (01) on PC6, PC7, PC8 and PC9 */
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7)) \
                 | (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9)) \
                 | (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the SPI1 pins on GPIO PA4 PA5 PA6 PA7
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_SPI1(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
  /* (1) Select AF mode (10) on PA4, PA5, PA6, PA7 */
  /* (2) AF0 for SPI1 signals */
  GPIOA->MODER = (GPIOA->MODER 
                  & ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | \
                      GPIO_MODER_MODER6 | GPIO_MODER_MODER7))\
                  | (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 |\
                     GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); /* (1) */
  GPIOA->AFR[0] = (GPIOA->AFR[0] & \
                   ~(GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5 |\
                     GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7)); /* (2) */
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

  /* Configure SPI1 in master */
  /* (1) Master selection, BR: Fpclk/256 (due to C27 on the board, SPI_CLK is set to the minimum)
         CPOL and CPHA at zero (rising first edge) */
  /* (2) Slave select output enabled, RXNE IT, 8-bit Rx fifo */
  /* (3) Enable SPI1 */
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR; /* (1) */
  SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_RXNEIE | SPI_CR2_FRXTH | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; /* (2) */
  SPI1->CR1 |= SPI_CR1_SPE; /* (3) */
 
  /* Configure IT */
  /* (4) Set priority for SPI1_IRQn */
  /* (5) Enable SPI1_IRQn */
  NVIC_SetPriority(SPI1_IRQn, 0); /* (4) */
  NVIC_EnableIRQ(SPI1_IRQn); /* (5) */ 
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the SPI2 pins on GPIO PB12 PB13 PB14 PB15
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_SPI2(void)
{
  /* Enable the peripheral clock of GPIOB */
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
  /* (1) Select AF mode (10) on PB12, PB13, PB14, PB15 */
  /* (2) AF0 for SPI2 signals */
  GPIOB->MODER = (GPIOB->MODER 
                 & ~(GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | \
                     GPIO_MODER_MODER14 | GPIO_MODER_MODER15))\
                 | (GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1|\
                    GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1); /* (1) */
  GPIOB->AFR[1] = (GPIOB->AFR[1] & \
                   ~(GPIO_AFRH_AFRH4 | GPIO_AFRH_AFRH5 |\
                     GPIO_AFRH_AFRH6 | GPIO_AFRH_AFRH7)); /* (2) */
}

/**
  * @brief  This function configures SPI2.
  * @param  None
  * @retval None
  */
__INLINE void Configure_SPI2(void)
{
   /* Enable the peripheral clock SPI2 */
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

  /* Configure SPI2 in slave */
  /* nSS hard, slave, CPOL and CPHA at zero (rising first edge) */
  /* (1) RXNE IT, 8-bit Rx fifo */
  /* (2) Enable SPI2 */
  SPI2->CR2 = SPI_CR2_RXNEIE | SPI_CR2_FRXTH | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; /* (1) */
  SPI2->CR1 |= SPI_CR1_SPE; /* (2) */

  /* Configure IT */
  /* (3) Set priority for SPI2_IRQn */
  /* (4) Enable SPI2_IRQn */
  NVIC_SetPriority(SPI2_IRQn, 0); /* (3) */
  NVIC_EnableIRQ(SPI2_IRQn); /* (4) */   
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
  if((SPI1->SR & SPI_SR_TXE) == SPI_SR_TXE) /* Test Tx empty */
  {
    *(uint8_t *)&(SPI2->DR) = SPI2_DATA; /* To test SPI1 reception */
    *(uint8_t *)&(SPI1->DR) = SPI1_DATA; /* Will inititiate 8-bit transmission if TXE */
  }
}

/**
  * @brief  This function handles SPI1 interrupt request.
  * @param  None
  * @retval None
  */
void SPI1_IRQHandler(void)
{
  uint8_t SPI1_Data = 0;
	
  if((SPI1->SR & SPI_SR_RXNE) == SPI_SR_RXNE)
  {
    SPI1_Data = (uint8_t)SPI1->DR; /* receive data, clear flag */
  
    if(SPI1_Data == SPI2_DATA)
    {
      GPIOC->ODR ^= GPIO_ODR_9; /* toggle green LED */
    }
    else
    {
      GPIOC->BSRR = GPIO_BSRR_BS_6; /* lit red LED */
    }
  }
  else
  {
    GPIOC->BSRR = GPIO_BSRR_BS_6; /* lit red LED */
    NVIC_DisableIRQ(SPI1_IRQn); /* Disable SPI1_IRQn */
  }
}

/**
  * @brief  This function handles SPI2 interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
  uint8_t SPI2_Data = 0;
	
  if((SPI2->SR & SPI_SR_RXNE) == SPI_SR_RXNE)
  {
    SPI2_Data = (uint8_t)SPI2->DR; /* Receive data, clear flag */
  
    if(SPI2_Data == SPI1_DATA)
    {
      GPIOC->ODR ^= GPIO_ODR_7; /* toggle blue LED */
    }
    else
    {
      GPIOC->BSRR = GPIO_BSRR_BS_8; /* lit orange LED */
    }
  }
  else
  {
    GPIOC->BSRR = GPIO_BSRR_BS_8; /* lit orange LED */
    NVIC_DisableIRQ(SPI2_IRQn); /* Disable SPI2_IRQn */
  }
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
