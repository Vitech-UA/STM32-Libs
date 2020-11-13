/**
  ******************************************************************************
  * @file    01_Configuration/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the timers 16 and 17 
  *          and PB9 to drive a high sink LED. 
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
  - RCC
  - TIM16/TIM17
  - IRTIM
  - GPIO PB9 for IR_OUT

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
    - This example configures the TIM16 and TIM17 in order to generate 
      a PWM edge aligned on OC1 (channel 1)of each timer.
      The GPIO PB9, corresponding to IR_OUT, is configured as alternate function 
      and the AF0 is selected.
    - To test this example, the user must monitor the signal on PB9.

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

/* Define the Timer to be configured */
#define TIM_ENV TIM16
#define TIM_ENV_BASE TIM16_BASE
#define TIM_ENV_IRQn TIM16_IRQn
#define TIM_ENV_IRQHandler TIM16_IRQHandler

#define TIM_CAR TIM17
#define TIM_CAR_BASE TIM17_BASE

#define  RC5HIGHSTATE     ((uint32_t )0x02)   /* RC5 high level definition*/
#define  RC5LOWSTATE      ((uint32_t )0x01)   /* RC5 low level definition*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t RC5_RealFrameLength = 14;
uint8_t RC5_GlobalFrameLength = 64;
uint16_t RC5_addr = 23;
uint16_t RC5_innstruction = 1;
uint32_t ManchesterCodedMsg;
uint8_t SendOperationReady = 0;
volatile uint8_t SendOperationCompleted = 0;
uint8_t BitsSentCounter = 0;
uint16_t frame;

/* Private function prototypes -----------------------------------------------*/
void ConfigureTIM16_17ForIRTIM(void);
uint32_t RC5_ManchesterConvert(uint16_t RC5_BinaryFrameFormat);
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
  ConfigureTIM16_17ForIRTIM();
  while (1)  
  {
    frame = 0x3000 | (RC5_addr << 6) | RC5_innstruction; /* Prepare the frame to send */
    ManchesterCodedMsg =  RC5_ManchesterConvert(frame); /* Code the message with Manchester code */
    SendOperationReady = 1; /* set the ready bit */
    /* Enable and reset TIM_ENV */
    /* (1) Force update generation (UG = 1) */
    /* (2) Enable counter (CEN = 1) */
    TIM_ENV->EGR |= TIM_EGR_UG; /* (1) */
    TIM_ENV->CR1 |= TIM_CR1_CEN; /* (2) */
    
    while (!SendOperationCompleted) /* wait for the sending completion */
    {
            __WFI();
    }
    TIM_ENV->CR1 &= (uint16_t)(~TIM_CR1_CEN); /* Disable IRQ on TIM_ENV */
  }        
}


/**
  * @brief  This function configures the TIM16 and 17 as PWM mode 1
  *         and enables the peripheral clock on both timers and on GPIOB.
  *         It configures GPIO PB9 as Alternate function for IR_OUT
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIM16_17ForIRTIM(void)
{
  /* (1) Enable the peripheral clocks of Timer 16 and 17 and SYSCFG */
  /* (2) Enable the peripheral clock of GPIOB */
  /* (3) Select alternate function mode on GPIOB pin 9 */
  /* (4) Select AF0 on PB9 in AFRH for IR_OUT (reset value) */
  /* (5) Enable the high sink driver capability by setting I2C_PB9_FM+ bit 
         in SYSCFG_CFGR1 */
  
  RCC->APB2ENR |= RCC_APB2ENR_TIM16EN | RCC_APB2ENR_TIM17EN   
                | RCC_APB2ENR_SYSCFGCOMPEN; /* (1) */
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN; /* (2) */  
  GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER9)
               | GPIO_MODER_MODER9_1; /* (3) */
  //GPIOB->AFR[1] &= ~(0x0F << ((9 - 8) * 4)); /* (4) */
   
  SYSCFG->CFGR1 |= SYSCFG_CFGR1_I2C_FMP_PB9; /* (5) */
                     
  /* Configure TIM_CAR as carrier signal */
  /* (1) Set prescaler to 1, so APBCLK i.e 48MHz */ 
  /* (2) Set ARR = 1333, as timer clock is 48MHz the frequency is 36kHz */
  /* (3) Set CCRx = 1333/4, , the signal will bhave a 25% duty cycle */
  /* (4) Select PWM mode 1 on OC1  (OC1M = 110),
         enable preload register on OC1 (OC1PE = 1) */
  /* (5) Select active high polarity on OC1 (CC1P = 0, reset value),
         enable the output on OC1 (CC1E = 1)*/
  /* (6) Enable output (MOE = 1)*/
  TIM_CAR->PSC =0; /* (1) */
  TIM_CAR->ARR = 1333; /* (2) */
  TIM_CAR->CCR1 = (uint16_t)(1333/4); /* (3) */
  TIM_CAR->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE; /* (4) */
  TIM_CAR->CCER |= TIM_CCER_CC1E; /* (5) */
  TIM_CAR->BDTR |= TIM_BDTR_MOE; /* (6) */
  
  /* Configure TIM_ENV is the modulation enveloppe */
  /* (1) Set prescaler to 1, so APBCLK i.e 48MHz */ 
  /* (2) Set ARR = 42627, as timer clock is 48MHz the period is 888 us */
  /* (3) Select Forced inactive on OC1  (OC1M = 100) */
  /* (4) Select active high polarity on OC1 (CC1P = 0, reset value),
         enable the output on OC1 (CC1E = 1)*/
  /* (5) Enable output (MOE = 1)*/
  TIM_ENV->PSC = 0; /* (1) */
  TIM_ENV->ARR = 42627; /* (2) */
  TIM_ENV->CCMR1 |= TIM_CCMR1_OC1M_2; /* (3) */
  TIM_ENV->CCER |= TIM_CCER_CC1E; /* (4) */
  TIM_ENV->BDTR |= TIM_BDTR_MOE; /* (5) */
  TIM_ENV->DIER |= TIM_DIER_UIE;

  /* Enable and reset TIM_CAR only */
  /* (1) Enable counter (CEN = 1)
         select edge aligned mode (CMS = 00, reset value)
         select direction as upcounter (DIR = 0, reset value) */  
  /* (2) Force update generation (UG = 1) */
  TIM_CAR->CR1 |= TIM_CR1_CEN; /* (1) */
  TIM_CAR->EGR |= TIM_EGR_UG; /* (2) */

  /* Configure TIM_ENV interrupt */
  /* (1) Enable Interrupt on TIM_ENV */
  /* (2) Set priority for TIM_ENV */
  NVIC_EnableIRQ(TIM_ENV_IRQn); /* (1) */
  NVIC_SetPriority(TIM_ENV_IRQn,0); /* (2) */
}


/**
  * @brief  Convert the RC5 frame from binary to Manchester Format.
  * @param  RC5_BinaryFrameFormat : the RC5 frame in binary format.
  * @retval The RC5 frame in Manchester format.
  */
uint32_t RC5_ManchesterConvert(uint16_t RC5_BinaryFrameFormat)
{
  uint32_t i=0;
  uint16_t Mask = 1;
  uint16_t bit_format = 0;
  uint32_t ConvertedMsg =0;
  
  for (i=0; i < RC5_RealFrameLength; i++)
  {
    bit_format =((((uint16_t)(RC5_BinaryFrameFormat))>>i)& Mask)<<i;
    ConvertedMsg = ConvertedMsg << 2;
    
    if(bit_format != 0 ) /* Manchester 1 -|_  */
    {
      ConvertedMsg |= RC5HIGHSTATE;
    }
    else /* Manchester 0 _|-  */
    {
      ConvertedMsg |= RC5LOWSTATE;
    }
  }
  return (ConvertedMsg);
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

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f072xb.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles TIM_ENV interrupt request.
  *         This interrupt subroutine computes the laps between 2 rising edges 
  *         on T1IC. This laps is stored in the "Counter" variable.
  * @param  None
  * @retval None
  */
void TIM_ENV_IRQHandler(void)
{
  uint8_t bit_msg = 0;
  
  if((SendOperationReady == 1) && (BitsSentCounter < (RC5_GlobalFrameLength * 2)))
  {
    if (BitsSentCounter < 32)
    {
      SendOperationCompleted = 0x00;
      bit_msg = (uint8_t)((ManchesterCodedMsg >> BitsSentCounter)& 1);
      
      if (bit_msg== 1)
      {
        TIM_ENV->CCMR1 |= TIM_CCMR1_OC1M_0; /* Force active level - OC1REF is forced high */
      }
      else
      {
        TIM_ENV->CCMR1 &= (uint16_t)(~TIM_CCMR1_OC1M_0); /* Force inactive level - OC1REF is forced low */
      }
    }
    BitsSentCounter++;
  }
  else
  {
    SendOperationCompleted = 0x01;
    SendOperationReady = 0;
    BitsSentCounter = 0;
  }
  /* Clear TIM_ENV update interrupt */
  TIM_ENV->SR &= (uint16_t)(~TIM_SR_UIF);
}
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
