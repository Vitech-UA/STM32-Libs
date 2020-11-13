/**
  ******************************************************************************
  * @file    04_HSI_Calibration/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to calibrate the HSI
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
    - RCC
    - GPIO PC8,9 for LEDs
    - GPIO PA8 for MCO
    - GPIO PF0 for HSE but no configuration needed
    - TIM14
    - SYSTICK (to manage led blinking)

 ===============================================================================
                    ##### How to use this example #####
 ===============================================================================
    - this file must be inserted in a project containing the following files:
      o stm32f0xx_it.c, system_stm32f0xx.c, startup_stm32f072xb.s
      o error_mngt.h for error management and external definitions
      o stm32f0xx.h to get the register definitions
      o stm32f0xx_it.h to get interrupt definitions
      o CMSIS files
 
 ===============================================================================
                   ##### How to test this example #####
 ===============================================================================
    - On the discovery board ensure that SB19 is closed and SB17 is open.
    - The HSE on OSC_IN is provided by the MCO of the embedded ST-Link  
      and is clocked at 8MHz.
    - The HSI is used as PLL clock source.
    - This examples starts the HSE and use HSE/32 as calibration clock thru the IC1F 
      of Timer 14.
      The TIM14 interrupt subroutine computes the period of the signal on 
      TIM14 IC1F.
      This period is compared with the theorical one. While it is out of a
      defined error range, the trimming bits are tuned to set the HSI in the 
      defined error range.
    - The HSI clock is output on MCO pin in order to be able to check the tuning 
      of its value thru an oscilloscope.
    - While the measured period is out of the trimming bit capability, an error 
      is shown by blinking the orange led. 
      The green LED blinks once HSI is correctly calibrated.

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
/* Delay value : short one is used for the error coding, long one in case of no error
   or between two bursts */
#define SHORT_DELAY 200
#define LONG_DELAY 1000


/* Error codes used to make the orange led blinking */
#define ERROR_TRIMMING_ON_GOING 0x01
#define ERROR_WRONG_MEASURE 0x02
#define ERROR_WRONG_IT 0x04

/* Clock constant definitions */
#define HSE 8
#define DIVIDER_ON_TI1 32 

#define PLL_INPUT 4
#define PLL_FACTOR    12

#define IC1_PRESCALER 4
#define COUNTER_TARGET ((DIVIDER_ON_TI1 * PLL_INPUT * PLL_FACTOR * IC1_PRESCALER)/HSE)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t error = 0xFF; /* Initialized at 0xFF and set in the TIM14 interrupt subroutine */
uint16_t gap = 0; /* Initialized at 0 and used to synchronize the IC laps computation */
uint16_t counter0; /* Used to store the first rising edge IC counter value */

/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void ConfigureClockController(void);
void ConfigureTIM14asInputCapture(void);
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
  SysTick_Config(48000); /* 1ms config */  
  ConfigureClockController();
  ConfigureTIM14asInputCapture();
   
  while (1) /* Infinite loop */
  {
  }
}


/**
  * @brief  This function enables the peripheral clocks on GPIO ports A and C,
  *         configures the Green LED pin on GPIO PC9,
  *         configures the orange LED pin on GPIO PC8,
  *         and configures GPIO PA8 as Alternate function and output high speed.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureGPIO(void)
{
  /* (1) Enable the peripheral clock of GPIOC and GPIOA */
  /* (2) Select output mode (01) on GPIOC pin 8 and 9 */
  /* (3) Select alternate function mode on GPIOA pin 8 */
  /* (4) Select high speed for GPIOA pin 8 (max 50MHz) - OSPEEDR8 = 11 */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN; /* (1) */
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9)) \
               | (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0); /* (2) */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER8)) | (GPIO_MODER_MODER8_1); /* (3) */
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8; /* (4) */
  
  /* MCO is AFR0 on PA8 and Alternate function 0 is selected by default 
    so no need to modify GPIOA_AFR */
}


/**
  * @brief  This function configures the RCC to output the HSI on MCO 
  *         and enables HSE from the MCO of the embedded ST-Link
  * @param  None
  * @retval None
  */
__INLINE void ConfigureClockController(void)
{
  /* (1) Select System clock to be output on the MCO without prescaler */
  /* (2) Set HSEBYP to use the external clock instead of an oscillator
         Enable the HSE */
  /* (3) Check HSE is ready */
  RCC->CFGR |= RCC_CFGR_MCO_HSI; /* (1) */
  RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON; /* (2) */
  while ((RCC->CR & RCC_CR_HSERDY) == 0) /* (3) */
  {
    /* For robust implementation, add here time-out management */
  }  
}


/**
  * @brief  This function configures the TIM14 as input capture 
  *         and enables the interrupt on TIM14
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIM14asInputCapture(void)
{
  /* (1) Enable the peripheral clock of Timer 14 */
  /* (2) Select the active input TI1,Program the input filter, and prescaler */
  /* (3) Enable interrupt on Capture/Compare */
  RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; /* (1) */
  TIM14->CCMR1 |= TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1 \
                | TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1PSC_1; /* (2)*/
  TIM14->DIER |= TIM_DIER_CC1IE; /* (3) */

  /* Configure NVIC for TIM14 */
  /* (4) Enable Interrupt on TIM14 */
  /* (5) Set priority for TIM14 */
  NVIC_EnableIRQ(TIM14_IRQn); /* (4) */
  NVIC_SetPriority(TIM14_IRQn,0); /* (5) */
  
  /* (6) Select HSE/32 as input on TI1 */
  /* (7) Enable counter */
  /* (8) Enable capture */
  TIM14->OR |= TIM14_OR_TI1_RMP_1; /* (6) */
  TIM14->CR1 |= TIM_CR1_CEN; /* (7) */
  TIM14->CCER |= TIM_CCER_CC1E; /* (8) */
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
  * @brief  This function handles TIM14 interrupt request.
  *         This interrupt subroutine computes the laps between 2 rising edges on T1IC
  *         This laps is stored in the "gap" variable and is compared 
  *         versus the counter target which has been computed using the PLL,  
  *         the input capture signal frequency and its prescaler.
  *         The HSI trimming is tuned to be in the counter target range +/- the half of the prescaler.
  * @param  None
  * @retval None
  */
void TIM14_IRQHandler(void)
{
uint16_t trim_value;
uint16_t counter1;
  
/*
  
*/
  if ((TIM14->SR & TIM_SR_CC1IF) != 0)
  {
    if ((TIM14->SR & TIM_SR_CC1OF) != 0)  /* Check the overflow */
    {
      gap = 0;  /* Reinitialize the laps computing */
      TIM14->SR &= ~(TIM_SR_CC1OF | TIM_SR_CC1IF); /* Clear the flags */
      return;
    }
    if (gap == 0) /* Test if it is the first rising edge */
    {
      counter0 = TIM14->CCR1; /* Read the capture counter which clears the CC1ICF */
      gap = 1; /* Indicate that the first rising edge has yet been detected */
    }
    else
    {
      trim_value = (RCC->CR & RCC_CR_HSITRIM)>>3; /* Store the trimming value */
      counter1 = TIM14->CCR1; /* Read the capture counter which clears the CC1ICF */
      if (counter1 > counter0) /* Check capture counter overflow */
      {
        gap = counter1 - counter0;
      }
      else
      {
        gap = counter1 + 0xFFFF - counter0 + 1;
      }
      if ((gap < COUNTER_TARGET - 16*IC1_PRESCALER) || (gap > COUNTER_TARGET + 15*IC1_PRESCALER )) /* Check laps reliability */
      {
        gap = 0; /* Reinitialize the laps computing */
        error =  ERROR_WRONG_MEASURE; 
      }
      else if (gap > (COUNTER_TARGET + IC1_PRESCALER/2)) /* Check if measured HSI is too low */
      {
        trim_value -= ((gap - (COUNTER_TARGET+ IC1_PRESCALER -1))/IC1_PRESCALER) + 1; /* Decrease the trimming */
        error =  ERROR_TRIMMING_ON_GOING; /* Report a warning */
      }
      else if (gap < (COUNTER_TARGET - IC1_PRESCALER/2)) /* Check if measured HSI is too high */
      {
        trim_value += ((COUNTER_TARGET - IC1_PRESCALER + 1 - gap)/IC1_PRESCALER)+ 1; /* Increase the trimming */
        error =  ERROR_TRIMMING_ON_GOING; /* Report a warning */
      }
      else
      {
         error = 0; /* The HSI is in the range */
      }
      
      RCC->CR = (RCC->CR & ~RCC_CR_HSITRIM) | (trim_value << 3) ; /* Restore the trimming bits */
      gap = 0; /* Reset the gap to restart a lap computing */
    }
    
  }
  else
  {
    error = ERROR_WRONG_IT; /* Report an error */
  }
}
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
