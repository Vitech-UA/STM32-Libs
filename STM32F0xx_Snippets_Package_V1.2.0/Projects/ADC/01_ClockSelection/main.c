/**
  ******************************************************************************
  * @file    01_ClockSelection/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the ADC and the RCC to 
  *          select the ADC clock and run basic acquisitions.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - ADC
   - GPIO PC8 and PC9

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
    - The code configures the clock to run the ADC.    
    - An ADC calibration is performed
    - The acquisition is configured according to ADC_CLOCK definition
    - An acquisition is repeatedly launched 
    - Once the calibration is completed the green led blinks.
    - The ADC result is not read and processed in this example.
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
/* Delay value : short one is used for the error coding, long one (~1s) in case 
   of no error or between two bursts */
#define SHORT_DELAY 200
#define LONG_DELAY 1000


/* Define used below to easily configure the ADC clock */
#define ADC_PRESCALER_DIV2 ADC_CFGR2_CKMODE_0
#define ADC_PRESCALER_DIV4 ADC_CFGR2_CKMODE_1


/* Preprocessor variable to select the ADC clock
   If HSI14 is not selected as ADC clock, HSI14 must be commented
   HSI14 select the ADC dedicated asynchronous 14MHz RC oscillator
   RCC_CFGR_PPRE_DIVx prescale the AHB clock by x to get the peripheral clock (PCLK)
   It is user responsibility to ensure the ADC clock is 14MHz max 
   The ADC interface will divide the PCLK by 2 or by 4 depending on ADC_PRESCALER */
#define HSI14 0
#define ADC_CLOCK HSI14
//#define ADC_CLOCK RCC_CFGR_PPRE_DIV2
//#define ADC_CLOCK RCC_CFGR_PPRE_DIV4
//#define ADC_CLOCK RCC_CFGR_PPRE_DIV8
//#define ADC_CLOCK RCC_CFGR_PPRE_DIV16

#ifdef  HSI14
#define ADC_PRESCALER 0
#else
#define ADC_PRESCALER ADC_PRESCALER_DIV2
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t error = 0xFF;  //initialized at 0 and modified by the functions 
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void SetClockForADC(void);
void CalibrateADC(void);
void ConfigureADC(void);
void EnableADC(void);
void DisableADC(void);
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
  ConfigureGPIO();
  SysTick_Config(48000);/* 1ms config */
  SetClockForADC();
  CalibrateADC();
  EnableADC();
  ConfigureADC();
  error = 0;
  while (1) /* Loop never exited */
  {
    /* Performs the AD converion */
    ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) /* wait end of conversion */
    {
      /* For robust implementation, add here time-out management */
    }
    /* Add ADC data management here */
  }
}

/**
  * @brief  This function enables the peripheral clocks on GPIO ports C,
  *         configures the Green LED pin on GPIO PC9,
  *         configures the orange LED pin on GPIO PC8,
  *         and enables GPIO clock
  * @param  None
  * @retval None
  */
__INLINE void ConfigureGPIO(void)
{  
  /* (1) Enable the peripheral clock of GPIOC */ 
  /* (2) Reset the mode and select output mode on GPIOC pin 8 and 9 */ 
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  /* (1) */
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9))\
               | (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);  /* (2) */
}

/**
  * @brief  This function enables the clock in the RCC for the ADC
  *         and select the clock according to ADC_CLOCK
  * @param  None
  * @retval None
  */
__INLINE void SetClockForADC(void)
{
  /* (1) Enable the peripheral clock of the ADC */
  /* (2) Start HSI14 RC oscillator */ 
  /* (3) Wait HSI14 is ready */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* (1) */
#ifdef HSI14
  RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
  while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
  {
    /* For robust implementation, add here time-out management */
  }
#else /* Use synchronous clock */
  RCC->CR = (RCC->CR & (~RCC_CFGR_PPRE)) | ADC_CLOCK;
#endif
}


/**
  * @brief  This function performs a self-calibration of the ADC
  * @param  None
  * @retval None
  */
__INLINE void  CalibrateADC(void)
{
  /* (1) Ensure that ADEN = 0 */
  /* (2) Clear ADEN */ 
  /* (3) Launch the calibration by setting ADCAL */
  /* (4) Wait until ADCAL=0 */
  if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
  {
    ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);  /* (2) */  
  }
  ADC1->CR |= ADC_CR_ADCAL; /* (3) */
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (4) */
  {
    /* For robust implementation, add here time-out management */
  }
}

/**
  * @brief  This function configure the ADC to convert the internal reference voltage (VRefInt)
  *         The conversion frequency is 14MHz 
  * @param  None
  * @retval None
  */
__INLINE void ConfigureADC(void)
{
  /* (1) Select the clock mode (refer ADC_PRESCALER definition :
         HSI14 by writing 00 in CKMODE (reset value)
         DIV2 or DIV 4 */ 
  /* (2) Select the auto off mode */
  /* (3) Select CHSEL17 for VRefInt */
  /* (4) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than 17.1us */
  /* (5) Wake-up the VREFINT (only for VBAT, Temp sensor and VRefInt) */
  ADC1->CFGR2 |= ADC_PRESCALER; /* (1) */   
  ADC1->CFGR1 |= ADC_CFGR1_AUTOFF; /* (2) */
  ADC1->CHSELR = ADC_CHSELR_CHSEL17; /* (3) */
  ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* (4) */
  ADC->CCR |= ADC_CCR_VREFEN; /* (5) */
}


/**
  * @brief  This function enables the ADC
  * @param  None
  * @retval None
  */
__INLINE void EnableADC(void)
{
  /* (1) Enable the ADC */
  /* (2) Wait until ADC ready */
  do 
  {
    /* For robust implementation, add here time-out management */
		ADC1->CR |= ADC_CR_ADEN; /* (1) */
  }while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (2) */;
}


/**
  * @brief  This function disables the ADC
  * @param  None
  * @retval None
  */
__INLINE void DisableADC(void)
{
  /* (1) Ensure that no conversion on going */
  /* (2) Stop any ongoing conversion */
  /* (3) Wait until ADSTP is reset by hardware i.e. conversion is stopped */
  /* (4) Disable the ADC */
  /* (5) Wait until the ADC is fully disabled */
  if ((ADC1->CR & ADC_CR_ADSTART) != 0) /* (1) */
  {
    ADC1->CR |= ADC_CR_ADSTP; /* (2) */
  }
  while ((ADC1->CR & ADC_CR_ADSTP) != 0) /* (3) */
  {
     /* For robust implementation, add here time-out management */
  }
  ADC1->CR |= ADC_CR_ADDIS; /* (4) */
  while ((ADC1->CR & ADC_CR_ADEN) != 0) /* (5) */
  {
    /* For robust implementation, add here time-out management */
  }  
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
  *         It toggles the green led if the action has been performed correctly
  *         and toggles the orange led coding the error number
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  static uint32_t long_counter = LONG_DELAY;
  static uint32_t short_counter = SHORT_DELAY;  
  static uint16_t error_temp = 0;
  
  if (long_counter-- == 0) 
  {
    if(error == 0)
    {
      /* the following instruction can only be used if no ISR modifies GPIOC ODR
         either by writing directly it or by using GPIOC BSRR or BRR 
         else a toggle mechanism must be implemented using GPIOC BSRR and/or BRR
      */
      GPIOC->ODR ^= (1<<9);//toggle green led on PC9
      long_counter = LONG_DELAY;
    }
    else if (error != 0xFF)
    {
      /* orange led blinks according to the code error value */
      error_temp = (error << 1) - 1;
      short_counter = SHORT_DELAY;
      long_counter = LONG_DELAY << 1;
      GPIOC->BSRR = (1<<8); //set orange led on PC8
      GPIOC->BRR = (1<<9); //switch off green led on PC9
    }
  }
  if (error_temp > 0)
  {
    if (short_counter-- == 0) 
    {
      GPIOC->ODR ^= (1 << 8); //toggle orange led
      short_counter = SHORT_DELAY;
      error_temp--;
    }  
  }
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f072xb.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
