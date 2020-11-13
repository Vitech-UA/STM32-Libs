/**
  ******************************************************************************
  * @file    01_ConfigurationTransmissionReception/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the GPIOs and bxCAN
  *          in order to transmit and receive in loopback mode.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - bxCAN
   - Systick
   - GPIO  PA0, PC8, PC9,
           PA11(CAN_RX), PA12(CAN_TX)
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
   - Launch the program
   - Press the user button
   - The orange LED toggles 
     (that's means a message is well transmit and received in loopback mode)
   - The transmit frame can be check with an oscilloscope on PA12
     (with SB20 fitted)
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
#define CMD_TOGGLE (0xDA)
#define CAN_ID_MASK (0xFF70U)
#define CAN_ID1 (0x651U)
#define CAN_ID2 (0x652U)
#define FILTER_LIST (0) /* 0: filter mode = identifier mask, 1: filter mode = identifier list */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Configure_GPIO_LED(void);
void Configure_GPIO_CAN(void);
void Configure_CAN(void);
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
  Configure_GPIO_CAN();
  Configure_CAN();
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
             - Configures the CAN pins on GPIO PA11 PA12
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_CAN(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
  /* GPIO configuration for CAN signals */
  /* (1) Select AF mode (10) on PA11 and PA12 */
  /* (2) AF4 for CAN signals */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12))\
                 | (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1); /* (1) */
  GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFRH3 | GPIO_AFRH_AFRH4))\
                  | (4 << (3 * 4)) | (4 << (4 * 4)); /* (2) */
}

/**
  * @brief  This function configures CAN.
  * @param  None
  * @retval None
  */
__INLINE void Configure_CAN(void)
{
  /* Enable the peripheral clock CAN */
  RCC->APB1ENR |= RCC_APB1ENR_CANEN;

  /* Configure CAN */
  /* (1) Enter CAN init mode to write the configuration */
  /* (2) Wait the init mode entering */
  /* (3) Exit sleep mode */
  /* (4) Loopback mode, set timing to 1Mb/s: BS1 = 4, BS2 = 3, prescaler = 6 */
  /* (5) Leave init mode */
  /* (6) Wait the init mode leaving */
  /* (7) Enter filter init mode, (16-bit + mask, filter 0 for FIFO 0) */
  /* (8) Acivate filter 0 */
  /* (9) Identifier list mode */
  /* (11) Set the Id list */
  /* (12) Set the Id + mask (all bits of standard id will care) */
  /* (13) Leave filter init */
  /* (14) Set FIFO0 message pending IT enable */
  CAN->MCR |= CAN_MCR_INRQ; /* (1) */
  while((CAN->MSR & CAN_MSR_INAK)!=CAN_MSR_INAK) /* (2) */
  { 
    /* add time out here for a robust application */
  }
  CAN->MCR &=~ CAN_MCR_SLEEP; /* (3) */
  CAN->BTR |= CAN_BTR_LBKM | 2 << 20 | 3 << 16 | 5 << 0; /* (4) */ 
  CAN->MCR &=~ CAN_MCR_INRQ; /* (5) */
  while((CAN->MSR & CAN_MSR_INAK)==CAN_MSR_INAK) /* (6) */
  { 
    /* add time out here for a robust application */
  }  
  CAN->FMR = CAN_FMR_FINIT; /* (7) */ 
  CAN->FA1R = CAN_FA1R_FACT0; /* (8) */
#if (FILTER_LIST)  
  CAN->FM1R = CAN_FM1R_FBM0; /* (9) */
  CAN->sFilterRegister[0].FR1 = CAN_ID2 << 5 | CAN_ID1 << (16+5); /* (10) */ 
#else  
  CAN->sFilterRegister[0].FR1 = CAN_ID1 << 5 | CAN_ID_MASK << 16; /* (11) */
#endif /* FILTER_LIST */
  
  CAN->FMR &=~ CAN_FMR_FINIT; /* (12) */
  CAN->IER |= CAN_IER_FMPIE0; /* (13) */
  
  /* Configure IT */
  /* (14) Set priority for CAN_IRQn */
  /* (15) Enable CAN_IRQn */
  NVIC_SetPriority(CEC_CAN_IRQn, 0); /* (16) */
  NVIC_EnableIRQ(CEC_CAN_IRQn); /* (17) */
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
  
  if((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0)/* check mailbox 0 is empty */
  {
    CAN->sTxMailBox[0].TDTR = 1; /* fill data length = 1 */
    CAN->sTxMailBox[0].TDLR = CMD_TOGGLE; /* fill 8-bit data */
    CAN->sTxMailBox[0].TIR = (uint32_t)(CAN_ID1 << 21 | CAN_TI0R_TXRQ); /* fill Id field and request a transmission */
  }
  else
  {
    CAN->TSR |= CAN_TSR_ABRQ0; /* abort transmission if not empty */
  }
}

/**
  * @brief  This function handles CAN interrupt request.
  * @param  None
  * @retval None
  */
void CEC_CAN_IRQHandler(void)
{
  uint32_t CAN_ReceiveMessage = 0;

  if((CAN->RF0R & CAN_RF0R_FMP0)!=0)/* check if a message is filtered and received by FIFO 0 */
  {
    CAN_ReceiveMessage = CAN->sFIFOMailBox[0].RDLR; /* read data */
    CAN->RF0R |= CAN_RF0R_RFOM0; /* release FIFO */
    if((CAN_ReceiveMessage & 0xFF) == CMD_TOGGLE)
    {
      GPIOC->ODR ^= GPIO_ODR_8; /* Toggle orange LED */
    }
  }
}


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
