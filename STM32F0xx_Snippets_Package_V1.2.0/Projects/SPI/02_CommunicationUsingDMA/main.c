/**
  ******************************************************************************
  * @file    02_CommunicationUsingDMA/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the GPIOs and SPI1(master)
  *          and SPI2 (slave) in full duplex communication in order to send and 
  *          receive bytes in DMA mode.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - SPI1, SPI2
   - DMA1
   - GPIO  PA0, PC6, PC7, PC8, PC9,
           PA4(SPI1_NSS), PA5(SPI1_SCK), PA6(SPI1_MISO), PA7(SPI1_MOSI), 
           PB12(SPI2_NSS), PB13(SPI2_SCK), PB14(SPI2_MISO), PB15(SPI2_MOSI)
   - RCC
   
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
   - The green and orange LEDs toggles (message transmit and receive)
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
#include "string.h" /* for strcmp */
#include "stdlib.h" /* for malloc */

/** @addtogroup STM32F0_Snippets
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint8_t stringtosendwSPI1[] = "DMA with SPI 1"; /* Both strings have same length */
const uint8_t stringtosendwSPI2[] = "And with SPI 2"; /* Receive and transmit are done in the same transaction */
uint8_t * stringtoreceivewSPI1;
uint8_t * stringtoreceivewSPI2;

/* Private function prototypes -----------------------------------------------*/
void Configure_GPIO_LED(void);
void Configure_GPIO_SPI1(void);
void Configure_DMA1_SPI1(void);
void Configure_SPI1(void);
void Configure_GPIO_SPI2(void);
void Configure_DMA1_SPI2(void);
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
  Configure_DMA1_SPI1();
  Configure_SPI1();
  Configure_GPIO_SPI2();
  Configure_DMA1_SPI2();
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
  * @brief  This function configures DMA for SPI1.
  * @param  None
  * @retval None
  */
__INLINE void Configure_DMA1_SPI1(void)
{
  /* Enable the peripheral clock DMA11 */
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  
  /* DMA1 Channel2 SPI1_RX config */
  /* (1) Peripheral address */
  /* (2) Memory address */
  /* (3) Data size */
  /* (4) Memory increment */
  /*     Peripheral to memory */
  /*     8-bit transfer */
  /*     Transfer complete IT */
  DMA1_Channel2->CPAR = (uint32_t)&(SPI1->DR); /* (1) */
  stringtoreceivewSPI1 = malloc(sizeof(stringtosendwSPI2));
  DMA1_Channel2->CMAR = (uint32_t)stringtoreceivewSPI1; /* (2) */
  DMA1_Channel2->CNDTR = sizeof(stringtosendwSPI2); /* (3) */
  DMA1_Channel2->CCR |= DMA_CCR_MINC | DMA_CCR_TCIE  | DMA_CCR_EN; /* (4) */
  
  /* DMA1 Channel3 SPI1_TX config */
  /* (5) Peripheral address */
  /* (6) Memory address */
  /* (7) Memory increment */
  /*     Memory to peripheral*/
  /*     8-bit transfer */
  DMA1_Channel3->CPAR = (uint32_t)&(SPI1->DR); /* (5) */
  DMA1_Channel3->CMAR = (uint32_t)stringtosendwSPI1; /* (6) */
  DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_DIR; /* (7) */
  
  /* Configure IT */
  /* (8) Set priority for DMA1_Channel2_3_IRQn */
  /* (9) Enable DMA1_Channel2_3_IRQn */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0); /* (8) */
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn); /* (9) */
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
  /* (2) TX and RX with DMA, slave select output enabled, RXNE IT, 8-bit Rx fifo */
  /* (3) Enable SPI1 */
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR; /* (1) */
  SPI1->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_SSOE | SPI_CR2_RXNEIE | SPI_CR2_FRXTH | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; /* (2) */
  SPI1->CR1 |= SPI_CR1_SPE; /* (3) */
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
  * @brief  This function configures DMA for SPI2.
  * @param  None
  * @retval None
  */
__INLINE void Configure_DMA1_SPI2(void)
{
  /* Enable the peripheral clock DMA11 */
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  
  /* DMA1 Channel4 SPI2_RX config */
  /* (1) Peripheral address */
  /* (2) Memory address */
  /* (3) Data size */
  /* (4) Memory increment */
  /*     Peripheral to memory */
  /*     8-bit transfer */
  /*     Transfer complete IT */
  DMA1_Channel4->CPAR = (uint32_t)&(SPI2->DR); /* (1) */
  stringtoreceivewSPI2 = malloc(sizeof(stringtosendwSPI1));
  DMA1_Channel4->CMAR = (uint32_t)stringtoreceivewSPI2; /* (2) */
  DMA1_Channel4->CNDTR = sizeof(stringtosendwSPI1); /* (3) */
  DMA1_Channel4->CCR |= DMA_CCR_MINC | DMA_CCR_TCIE  | DMA_CCR_EN; /* (4) */
  
  /* DMA1 Channel5 SPI2_TX config */
  /* (5) Peripheral address */
  /* (6) Memory address */
  /* (7) Memory increment */
  /*     Memory to peripheral*/
  /*     8-bit transfer */
  DMA1_Channel5->CPAR = (uint32_t)&(SPI2->DR); /* (5) */
  DMA1_Channel5->CMAR = (uint32_t)stringtosendwSPI2; /* (6) */
  DMA1_Channel5->CCR |= DMA_CCR_MINC | DMA_CCR_DIR; /* (7) */
  
  /* Configure IT */
  /* (8) Set priority for DMA1_Channel4_5_6_7_IRQn */
  /* (9) Enable DMA1_Channel4_5_6_7_IRQn */
  NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0); /* (8) */
  NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn); /* (9) */

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
  /* (1) TX and RX with DMA, RXNE IT, 8-bit Rx fifo */
  /* (2) Enable SPI2 */
  SPI2->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_RXNEIE | SPI_CR2_FRXTH | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; /* (1) */
  SPI2->CR1 |= SPI_CR1_SPE; /* (2) */
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
  
  /* start 8-bit transmission with DMA*/
  /* Prepare Slave */
  DMA1_Channel5->CCR &=~ DMA_CCR_EN;
  DMA1_Channel5->CNDTR = sizeof(stringtosendwSPI2); /* Data size */
  DMA1_Channel5->CCR |= DMA_CCR_EN;
  /* Prepare master */
  DMA1_Channel3->CCR &=~ DMA_CCR_EN;
  DMA1_Channel3->CNDTR = sizeof(stringtosendwSPI1); /* Data size */
  DMA1_Channel3->CCR |= DMA_CCR_EN;
}

/**
  * @brief  This function handles DMA1 channel 2 TC interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  if((DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2)
  {
    DMA1->IFCR |= DMA_IFCR_CTCIF2; /* Clear TC flag */

    if(strcmp((const char *)stringtosendwSPI2,(const char *)stringtoreceivewSPI1) == 0)
    {
      GPIOC->ODR ^= GPIO_ODR_9; /* toggle green LED */
    }
     else
    {
      GPIOC->BSRR = GPIO_BSRR_BS_6; /* lit red LED */
    }
    
    DMA1_Channel2->CCR &=~ DMA_CCR_EN;
    DMA1_Channel2->CNDTR = sizeof(stringtosendwSPI2); /* Data size */
    DMA1_Channel2->CCR |= DMA_CCR_EN;
  }
  else
  {
    GPIOC->BSRR = GPIO_BSRR_BS_6; /* lit red LED */
    NVIC_DisableIRQ(DMA1_Channel2_3_IRQn); /* Disable DMA1_Channel2_3_IRQn */
  }
}

/**
  * @brief  This function handles DMA1 channel 4 TC interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
  if((DMA1->ISR & DMA_ISR_TCIF4) == DMA_ISR_TCIF4)
  {
    DMA1->IFCR |= DMA_IFCR_CTCIF4; /* Clear TC flag */

    if(strcmp((const char *)stringtosendwSPI1,(const char *)stringtoreceivewSPI2) == 0)
    {
      GPIOC->ODR ^= GPIO_ODR_7; /* toggle orange LED */
    }
     else
    {
      GPIOC->BSRR = GPIO_BSRR_BS_8; /* lit orange LED */
    }
    
    DMA1_Channel4->CCR &=~ DMA_CCR_EN;
    DMA1_Channel4->CNDTR = sizeof(stringtosendwSPI1); /* Data size */
    DMA1_Channel4->CCR |= DMA_CCR_EN;
  }
  else
  {
    GPIOC->BSRR = GPIO_BSRR_BS_8; /* lit orange LED */
    NVIC_DisableIRQ(DMA1_Channel4_5_6_7_IRQn); /* Disable DMA1_Channel4_5_IRQn */
  }
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
