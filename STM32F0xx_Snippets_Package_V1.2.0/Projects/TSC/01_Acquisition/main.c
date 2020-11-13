/**
  ******************************************************************************
  * @file    01_Acquisition/main.c 
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
   - Systick
   - GPIO  PA0, PC8, PC9,
           PA7(Sampling capacitor, G2IO4), PA6(electrode, G2IO3)
   - TSC
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
   - The green LED lit (mode "ON", acquisitions are on going on the sensor)
   - Touch the 2nd element of the linear sensor
   - The orange LED lit while it is pressed
   - after few time (5s), if no activities on linear sensor, 
     reference adaption is computed each 100ms
   - The board goes to "OFF" when press on the user button
   - Press the user button, to set the mode "ON"
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
#define THRESHOLD (50)
#define NUMBER_OF_CALIBRATION (10)
#define SOFT_DELAY (30)
#define TIME_BEFORE_ADAPTATION (5000) /* 5s */
#define TIME_BETWEEN_ADAPTATION (100) /* 100ms */
#define REFERENCE_TAB_SIZE (3)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t Standby = 0; /* default Standby = 0 => ON, Standby = 1 => OFF */
uint8_t CalibrationDone = 0; /* By default => calibration ongoing */
uint32_t AcquisitionValue = 0;
uint32_t Reference = 0;
uint8_t Activities = 0;
volatile uint32_t Count = 0;
uint8_t ReferenceAdaptation = 0;

/* Private function prototypes -----------------------------------------------*/
void Configure_GPIO_LED(void);
void Configure_GPIO_TSC(void);
void Configure_TSC(void);
void Configure_GPIO_Button(void);
void Configure_EXTI(void);
void Configure_Systick(void);
void Process(void);
void SoftDelay(void);

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
  Configure_GPIO_TSC();
  Configure_TSC();
  Configure_GPIO_Button();
  Configure_EXTI();
  Configure_Systick();

  /* First acquisition */
  TSC->CR |= (1<<1); /* TSC_CR_START = 1 */
  
  /* Infinite loop */
  while (1)
  {
    /* Comment this function to perform only one acquisition */
    Process();
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
  
  /* lit green LED */
  GPIOC->BSRR = GPIO_BSRR_BS_9;
}

/**
  * @brief  This function :
             - Enables GPIO clock
             - Configures the TSC pins on GPIO PA6 PA7
  * @param  None
  * @retval None
  */
__INLINE void Configure_GPIO_TSC(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
  /* (1) Open drain for sampling */
  /* (2) PP for channel */
  /* (3) Select AF mode (10) on PA6, PA7 */
  /* (4) AF3 for TSC signals */
  GPIOA->OTYPER |= GPIO_OTYPER_OT_7; /* (1) */
  GPIOA->OTYPER &=~ GPIO_OTYPER_OT_6; /* (2) */
  GPIOA->MODER = (GPIOA->MODER  & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7))\
                | (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); /* (3) */
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7))\
                     |(3<<(4*6))|(3<<(4*7)); /* (4) */
}

/**
  * @brief  This function configures TSC.
  * @param  None
  * @retval None
  */
__INLINE void Configure_TSC(void)
{
  /* Enable the peripheral clock TSC */
  RCC->AHBENR |= RCC_AHBENR_TSEN;

  /* Configure TSC */
  /* With a Charge transfer cycle around 2.5µs */
  /* (1) Select fPGCLK = fHCLK/32
         Set pulse high = 2xtPGCLK,Master
         Set pulse low = 2xtPGCLK 
         Set Max count value = 16383 pulses
         Enable TSC */
  /* (2) Disable hysteresis */
  /* (3) Enable end of acquisition IT */
  /* (4) Sampling enabled, G2IO4 */
  /* (5) Channel enabled, G2IO3 */
  /* (6) Enable group, G2 */
  TSC->CR = TSC_CR_PGPSC_2 | TSC_CR_PGPSC_0 | TSC_CR_CTPH_0 | TSC_CR_CTPL_0 
	  | TSC_CR_MCV_2 | TSC_CR_MCV_1 | TSC_CR_TSCE; /* (1) */
  TSC->IOHCR &= (uint32_t)(~(TSC_IOHCR_G2_IO4 | TSC_IOHCR_G2_IO3)); /* (2) */
  TSC->IER = TSC_IER_EOAIE; /* (3) */
  TSC->IOSCR = TSC_IOSCR_G2_IO4; /* (4) */
  TSC->IOCCR = TSC_IOCCR_G2_IO3; /* (5) */
  TSC->IOGCSR |= TSC_IOGCSR_G2E; /* (5) */
  
  /* Configure IT */
  /* (4) Set priority for TSC_IRQn */
  /* (5) Enable TSC_IRQn */
  NVIC_SetPriority(TSC_IRQn, 0); /* (4) */
  NVIC_EnableIRQ(TSC_IRQn); /* (5) */ 
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

/**
  * @brief  This function configures Systick.
  * @param  None
  * @retval None
  */
__INLINE void Configure_Systick(void)
{
  SysTick_Config(48000); /* 1ms config */
}

/**
  * @brief  This function processes the TSC value.
  * @param  None
  * @retval None
  */
void Process(void)
{
  static uint32_t NumberOfCalibration=0;
  uint8_t RefIndex=0;
  static uint8_t RefIndexStatic=0;
  static uint32_t ReferenceTab[REFERENCE_TAB_SIZE];
  
  if(!Standby) /* Acquisition mode */
  {
    if(AcquisitionValue) /* check if there is a new acquisition value */
    {
      if(CalibrationDone) /* check if the calibration is done */
      {
        if((AcquisitionValue + THRESHOLD) < Reference) /* Touch detected */
        {
          GPIOC->BSRR = GPIO_BSRR_BS_8; /* Lit orange LED */
          Activities = 1;
        }
        else if(AcquisitionValue > (Reference + THRESHOLD)) /* Need recalibration */
        {
          GPIOC->BSRR = GPIO_BSRR_BR_8; /* Off orange LED */
          Activities = 1;
          CalibrationDone = 0; /* restart calibration */
          Reference = 0; /* Reset reference */
        }
        else /* no touch detected */
        {
          GPIOC->BSRR = GPIO_BSRR_BR_8; /*  Off orange LED */
          
          /* Reference adaptation */
          if(ReferenceAdaptation)
          {
            ReferenceAdaptation=0;
            RefIndexStatic%=REFERENCE_TAB_SIZE;
            ReferenceTab[RefIndexStatic++] = AcquisitionValue;
            
            for(RefIndex=0;RefIndex<REFERENCE_TAB_SIZE;RefIndex++)
            {
               Reference += ReferenceTab[RefIndex];
            }
            Reference /= (REFERENCE_TAB_SIZE + 1);
          }
        }
      }
      else /* Calibration */
      {
        if(NumberOfCalibration < NUMBER_OF_CALIBRATION)
        {
          Reference += AcquisitionValue;
          NumberOfCalibration++;
        }
        else if(NumberOfCalibration == NUMBER_OF_CALIBRATION)
        {
          Reference += AcquisitionValue;
          Reference /= (NUMBER_OF_CALIBRATION + 1); /* Compute reference */
          NumberOfCalibration = 0; /* Reset number of calibration for nex time */
          CalibrationDone = 1; /* Calibration Completed */
          
          /* Fill reference table */
          for(RefIndex=0;RefIndex<REFERENCE_TAB_SIZE;RefIndex++)
          {
             ReferenceTab[RefIndex] = Reference;
          }
        }
      }
      AcquisitionValue = 0; /* Reset Acquisition value */
      SoftDelay(); /* Wait to discharge sample capacitor before new acquisition */
      TSC->CR |= (1<<1) ;/* new acquisition, TSC_CR_START = 1 */
    }
  }
}

/**
  * @brief  This function is a delay software.
  * @param  None
  * @retval None
  */
void SoftDelay(void)
{
  Count = SOFT_DELAY;
  while(Count--);
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
  static uint32_t Counter=0;
  static uint32_t CounterRef=0;
  
  if((Activities)||(Standby))
  {
    Counter=0;
  }
  else if(Counter < TIME_BEFORE_ADAPTATION)
  {
    Counter++;
  }
  else if(Counter == TIME_BEFORE_ADAPTATION) /* 5s */
  {
    if(CounterRef++ == TIME_BETWEEN_ADAPTATION) /* 100ms */
    {
      CounterRef=0;
      ReferenceAdaptation=1;
    }
  }
}


/**
  * @brief  This function handles EXTI 0 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_1_IRQHandler(void)
{
  EXTI->PR |= 1;
  
  Standby = 1 - Standby; /* Toggle stanby mode */
  
  if(Standby)
  {
    GPIOC->BSRR = GPIO_BSRR_BR_8 | GPIO_BSRR_BR_9;/* Off LEDs */
    TSC->CR &= (uint32_t)(~(1<<1)); /* Stop acquisition, TSC_CR_START = 0 */
  }
  else
  {
    GPIOC->BSRR = GPIO_BSRR_BS_9; /* Lit green LED */
    Reference = 0; /* Reset reference */
    CalibrationDone = 0; /* To perform calibration */
    TSC->CR |= (1<<1); /* New acquisition, TSC_CR_START = 1 */
  }
}

/**
  * @brief  This function handles TSC interrupt request.
  * @param  None
  * @retval None
  */
void TSC_IRQHandler(void)
{
  /* End of acquisition flag */
  if((TSC->ISR & TSC_ISR_EOAF) == TSC_ISR_EOAF)
  {
    TSC->ICR = TSC_ICR_EOAIC; /* Clear flag */
    AcquisitionValue = TSC->IOGXCR[1]; /* Get G2 counter value */
  }
  else
  {
    NVIC_DisableIRQ(TSC_IRQn); /* Disable TS_IRQn */
  }
}


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
