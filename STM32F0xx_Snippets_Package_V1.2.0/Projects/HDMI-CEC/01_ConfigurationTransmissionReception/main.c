/**
  ******************************************************************************
  * @file    01_ConfigurationTransmissionReception/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the GPIOs and HDMI-CEC
  *          in order to transmit and receive.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC   
   - GPIO PA0, PC8, PC9, PB10(CEC line)
   - HDMI-CEC   
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
   - Connect a wire between PB10 to PB10 of an other discovery board
     with the example 02 loaded, there is a 4K7 PU on the line
     a 27K PU is recommended for normal CEC application
   - Connect GND of both boards
   - Launch the program
   - Press the user button
   - The green LED toggles (message transmit and receive) on the other board
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
#define ADDRESS_INITIATOR (0x0) /* Own address */
#define ADDRESS_DESTINATION (0x1) /* Destination */
#define CMD_TOGGLE_LED (0x22)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void Configure_GPIO_LED(void);
void Configure_GPIO_CEC(void);
void Configure_CEC(void);
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
  Configure_GPIO_CEC();
  Configure_CEC();
  Configure_GPIO_Button();
  Configure_EXTI();

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
             - Configures the CEC pins on GPIO PB10
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_CEC(void)
{
  /* Enable the peripheral clock of GPIOB */
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
  /* (1) Open drain for CEC signal */
  /* (2) Select AF mode (10) on PB10 */
  /* (3) AF0 for CEC signals */
  GPIOB->OTYPER |= GPIO_OTYPER_OT_10; /* (1) */
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER10)) \
                 | (GPIO_MODER_MODER10_1); /* (2) */
  GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(GPIO_AFRH_AFRH2)); /* (3) */
}

/**
  * @brief  This function configures CEC.
  * @param  None
  * @retval None
  */
__INLINE void Configure_CEC(void)
{
  /* Enable the peripheral clock CEC */
  RCC->APB1ENR |= RCC_APB1ENR_CECEN;

  /* Configure CEC */
  /* (1) OAR = 0x0001 => OA = 0x0 */
  /* (2) Receive byte interrupt enable, receive end interrupt enable */
  /* (3) CEC enable */
  CEC->CFGR = (0x0001<<16); /* (1) */
  CEC->IER = CEC_IER_RXBRIE|CEC_IER_RXENDIE; /* (2) */
  CEC->CR = CEC_CR_CECEN; /* (3) */

  /* Configure IT */
  /* (4) Set priority for CEC_CAN_IRQn */
  /* (5) Enable CEC_IRQn */
  NVIC_SetPriority(CEC_CAN_IRQn, 0); /* (4) */
  NVIC_EnableIRQ(CEC_CAN_IRQn); /* (5) */  
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
  
  /* Initiate the CEC message transmission sequence */
  /* (1) Set transmit IT */
  /* (2) Fill transmit data register with nibbles (initiator 0x0, destination 0x1) */
  /* (3) Set start of message bit */
  CEC->IER |= CEC_IER_TXBRIE; /* (1) */
  CEC->TXDR = (uint32_t)(ADDRESS_INITIATOR << 4 | ADDRESS_DESTINATION); /* (2) */
  CEC->CR |= CEC_CR_TXSOM; /* (3) */
}

/**
  * @brief  This function handles CEC interrupt request.
  * @param  None
  * @retval None
  */
void CEC_CAN_IRQHandler(void)
{
  uint8_t Received_Data=0;
  
  if((CEC->ISR & CEC_ISR_RXEND) == CEC_ISR_RXEND)
  {
    CEC->ISR = CEC_ISR_RXBR | CEC_ISR_RXEND; /* Reset flag */
    Received_Data = CEC->RXDR;
    if(Received_Data == CMD_TOGGLE_LED)
    {
      GPIOC->ODR ^= GPIO_ODR_9; /* toggle green LED */
    }
  }
  else if((CEC->ISR & CEC_ISR_RXBR) == CEC_ISR_RXBR)
  {
    CEC->ISR = CEC_ISR_RXBR; /* Reset flag */
    Received_Data = CEC->RXDR;
    if(Received_Data != (ADDRESS_INITIATOR | ADDRESS_DESTINATION << 4))
    {
      GPIOC->BSRR = GPIO_BSRR_BS_8; /* lit orange LED */
    }
  }
  else if((CEC->ISR & CEC_ISR_TXBR) == CEC_ISR_TXBR)
  {
    CEC->IER &=~ CEC_IER_TXBRIE; /* Reset Tx IT */
    CEC->CR |= CEC_CR_TXEOM; /* this is the last byte */
    CEC->TXDR = CMD_TOGGLE_LED;
  }
  else
  {
    GPIOC->BSRR = GPIO_BSRR_BS_8; /* lit orange LED */
    NVIC_DisableIRQ(CEC_CAN_IRQn); /* disable CEC_IRQn */
  }
}


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
