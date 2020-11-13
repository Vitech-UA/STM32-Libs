/**
  ******************************************************************************
  * @file    03_CommunicationUsingDMA/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the GPIOs and I2C 
  *          in order to receive with slave and transmit with master.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO PB6(I2C1_SCL),PB7(I2C1_SDA),PB10(I2C2_SCL),PB11(I2C2_SDA),PA0,PC8,PC9
   - I2C1 (slave), I2C2 (master)
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
   - Plug wires between PB6/PB10 and PB7/PB11, 4K7 PU are already on the board.
   - Launch the program
   - Press the user button to initiate a transmit request by master 
     then slave receivess a byte
   - The green and orange LEDs toggles if everything goes well

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
#define I2C1_OWN_ADDRESS (0x5A)
#define I2C_CMD_TOGGLE_ORANGE (0x81)
#define I2C_CMD_TOGGLE_GREEN (0x82)
#define SIZE_OF_DATA  (2)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t datatosend[SIZE_OF_DATA]={I2C_CMD_TOGGLE_ORANGE,I2C_CMD_TOGGLE_GREEN};
uint8_t datatoreceive[SIZE_OF_DATA];

/* Private function prototypes -----------------------------------------------*/
void Configure_GPIO_LED(void);
void Configure_GPIO_I2C1(void);
void Configure_DMA_I2C1(void);
void Configure_I2C1_Slave(void);
void Configure_GPIO_I2C2(void);
void Configure_DMA_I2C2(void);
void Configure_I2C2_Master(void);
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
  Configure_GPIO_I2C1();
  Configure_DMA_I2C1();
  Configure_I2C1_Slave();
  Configure_GPIO_I2C2();
  Configure_DMA_I2C2();
  Configure_I2C2_Master();
  Configure_GPIO_Button();
  Configure_EXTI();

  /* Initiate I2C sequence in button IRQ handler */
	
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
	
  /* Select output mode (01) on PC8 and PC9 */
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9)) \
                 | (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the I2C1 pins on GPIO PB6 PB7
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_I2C1(void)
{
  /* Enable the peripheral clock of GPIOB */
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
  /* (1) open drain for I2C signals */
  /* (2) AF1 for I2C signals */
  /* (3) Select AF mode (10) on PB6 and PB7 */
  GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7; /* (1) */
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7)) \
                  | (1 << ( 6 * 4 )) | (1 << (7 * 4)); /* (2) */
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7)) \
                 | (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); /* (3) */

}

/**
  * @brief  This function configures DMA for I2C1.
  * @param  None
  * @retval None
  */
__INLINE void Configure_DMA_I2C1(void)
{
  /* Enable the peripheral clock DMA1 */
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  
   /* DMA1 Channel2 I2C1_RX config */
  /* (4)  Peripheral address */
  /* (5)  Memory address */
  /* (6)  Data size */
  /* (7)  Memory increment */
  /*      Peripheral to memory*/
  /*      8-bit transfer */
  /*      Transfer complete IT */
  DMA1_Channel3->CPAR = (uint32_t)&(I2C1->RXDR); /* (4) */
  DMA1_Channel3->CMAR = (uint32_t)datatoreceive; /* (5) */
  DMA1_Channel3->CNDTR = SIZE_OF_DATA; /* (6) */
  DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_EN; /* (7) */
  
  /* Configure IT */
  /* (8) Set priority for DMA1_Channel2_3_IRQn */
  /* (9) Enable DMA1_Channel2_3_IRQn */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0); /* (8) */
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn); /* (9) */
}

/**
  * @brief  This function configures I2C1, slave.
  * @param  None
  * @retval None
  */
__INLINE void Configure_I2C1_Slave(void)
{
  /* Configure RCC for I2C1 */
  /* (1) Enable the peripheral clock I2C1 */
  /* (2) Use SysClk for I2C CLK */
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; /* (1) */
  RCC->CFGR3 |= RCC_CFGR3_I2C1SW; /* (2) */
  
  /* Configure I2C1, slave */
  /* (2) Timing register value is computed with the AN4235 xls file,
   fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns */
  /* (3) Periph enable, receive DMA enable */
  /* (4) 7-bit address = 0x5A */
  /* (5) Enable own address 1 */
  I2C1->TIMINGR = (uint32_t)0x00B00000; /* (2) */
  I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXDMAEN | I2C_CR1_ADDRIE; /* (3) */
  I2C1->OAR1 |= (uint32_t)(I2C1_OWN_ADDRESS << 1); /* (4) */
  I2C1->OAR1 |= I2C_OAR1_OA1EN; /* (5) */
  
  /* Configure IT */
  /* (7) Set priority for I2C1_IRQn */
  /* (8) Enable I2C1_IRQn */
  NVIC_SetPriority(I2C1_IRQn, 0); /* (7) */
  NVIC_EnableIRQ(I2C1_IRQn); /* (8) */
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the I2C2 pins on GPIO PB10 PB11
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_I2C2(void)
{
  /* Enable the peripheral clock of GPIOB */
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
  /* (1) Open drain for I2C signals */
  /* (2) AF1 for I2C signals */
  /* (3) Select AF mode (10) on PB10 and PB11 */
  GPIOB->OTYPER |= GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11; /* (1) */
  GPIOB->AFR[1] = (GPIOB->AFR[1] &~ (GPIO_AFRH_AFRH2 | GPIO_AFRH_AFRH3)) \
                  | (1 << (2 * 4)) | (1 << (3 * 4)); /* (2) */
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11)) \
                 | (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1); /* (3) */
  
}

/**
  * @brief  This function configures DMA for I2C2.
  * @param  None
  * @retval None
  */
__INLINE void Configure_DMA_I2C2(void)
{
  /* Enable the peripheral clock DMA1 */
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  
  /* DMA1 Channel4 I2C2_TX config */
  /* (1)  Peripheral address */
  /* (2)  Memory address */
  /* (3)  Memory increment */
  /*      Memory to peripheral */
  /*      8-bit transfer */
  DMA1_Channel4->CPAR = (uint32_t)&(I2C2->TXDR); /* (1) */
  DMA1_Channel4->CMAR = (uint32_t)datatosend; /* (2) */
  DMA1_Channel4->CCR |= DMA_CCR_MINC | DMA_CCR_DIR; /* (3) */
}

/**
  * @brief  This function configures I2C2, master.
  * @param  None
  * @retval None
  */
__INLINE void Configure_I2C2_Master(void)
{
  /* Enable the peripheral clock I2C2 */
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

  /* Configure I2C2, master */
  /* (1) Timing register value is computed with the AN4235 xls file,
   fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns */
  /* (2) Periph enable */
  /* (3) Slave address = 0x5A, write transfer, 2 bytes to transmit, autoend */
  I2C2->TIMINGR = (uint32_t)0x00B01A4B; /* (1) */
  I2C2->CR1 = I2C_CR1_PE | I2C_CR1_TXDMAEN; /* (2) */
  I2C2->CR2 =  I2C_CR2_AUTOEND | (SIZE_OF_DATA << 16) | (I2C1_OWN_ADDRESS<<1); /* (3) */
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

  /* start I2C master transmission sequence */
  if((I2C2->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE)) /* Check Tx empty */
  {
   // I2C2->TXDR = I2C_BYTE_TO_SEND; /* Byte to send */
    //I2C2->CR2 |= I2C_CR2_START; /* Go */
    /* start transmission with DMA */
    DMA1_Channel4->CCR &=~ DMA_CCR_EN;
    DMA1_Channel4->CNDTR = SIZE_OF_DATA;/* Data size */
    DMA1_Channel4->CCR |= DMA_CCR_EN;
    
    I2C2->CR2 |= I2C_CR2_START; /* Go */
  }
}

/**
  * @brief  This function handles I2C1 interrupt request.
  * @param  None
  * @retval None
  */
void I2C1_IRQHandler(void)
{
  uint32_t I2C_InterruptStatus = I2C1->ISR; /* Get interrupt status */
  
  if((I2C_InterruptStatus & I2C_ISR_ADDR) == I2C_ISR_ADDR)
  {
    I2C1->ICR |= I2C_ICR_ADDRCF; /* Address match event */
  }
}

/**
  * @brief  This function handles DMA1 channel 2 and 3 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  if((DMA1->ISR & DMA_ISR_TCIF3) == DMA_ISR_TCIF3)
  {
    DMA1->IFCR |= DMA_IFCR_CTCIF3;/* Clear TC flag */

    if(datatoreceive[0] == I2C_CMD_TOGGLE_ORANGE)
    {
      datatoreceive[0]=0;
      GPIOC->ODR ^= GPIO_ODR_8; /* Toggle orange LED */
    }
    
    if(datatoreceive[1] == I2C_CMD_TOGGLE_GREEN)
    {
      datatoreceive[1]=0;
      GPIOC->ODR ^= GPIO_ODR_9; /* toggle green LED */
    }
    
    DMA1_Channel3->CCR &=~ DMA_CCR_EN;
    DMA1_Channel3->CNDTR = SIZE_OF_DATA;/* Data size */
    DMA1_Channel3->CCR |= DMA_CCR_EN;
  }
  else
  {
    NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);/* Disable DMA1_Channel2_3_IRQn */
  }
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
