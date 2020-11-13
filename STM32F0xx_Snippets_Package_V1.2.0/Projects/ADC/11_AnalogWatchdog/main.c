/**
  ******************************************************************************
  * @file    11_AnalogWatchdog/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the ADC to check   
  *          if a channel remains within a configured voltage range. 
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - ADC on PA1, PB1, PC0 and VREFINT
   - HSI14 MHz for ADC
   - GPIO PC8 and PC9
   - Low power (WFI)
   - SYSTICK (to manage led blinking)

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
    - This example configures the ADC to convert 4 channels (CH1, 9, 10 and 17)
      but CH1,9 and 10 are not processed
    - The code launches a continuous conversion.
    - The CH17 i.e. internal reference voltage(VREFINT) is monitored thanks to 
      the analog watchdog.
    - A shift on VREFINT means a VDD shift.
    - The green led is toggled at each end of sequence, while the VREFINT is 
      in the configured range.
    - In case the Analog Watchdog detects an error, the orange led is switched on
      and the ADC continues to convert.
    - In case of failure, the orange led blinks coding the error type
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

/* Error codes used to make the orange led blinking */
#define WARNING_VDD_SHIFT 0x01
#define ERROR_UNEXPECTED_ADC_IT 0x02
#define ERROR_UNEXPECTED_EXT_IT 0x04

#define NUMBER_OF_ADC_CHANNEL 4

/* Internal voltage reference calibration value address */
#define VREFINT_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7BA))
#define VDD_CALIB ((uint16_t) (330))
#define VDD_APPLI ((uint16_t) (300))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t error = 0;  //initialized at 0 and modified by the functions 

uint16_t ADC_array[NUMBER_OF_ADC_CHANNEL]; //Array to store the values coming from the ADC and copied by DMA
uint32_t CurrentChannel; //index on the ADC_array
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void SetClockForADC(void);
void CalibrateADC(void);
void ConfigureADC(void);
void ConfigureGPIOforADC(void);
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
  
     PLL is set to x12 with  a PREDIV /2 so the system clock SYSCLK = 48MHz
     */

  ConfigureGPIO();
  SetClockForADC();
  CalibrateADC();
  ConfigureGPIOforADC();
  EnableADC();
  ConfigureADC();
  CurrentChannel = 0; /* Initialize the CurrentChannel */
  ADC1->CR |= ADC_CR_ADSTART; /* Start the ADC conversions */
      
  while (error < ERROR_UNEXPECTED_ADC_IT) /* loop till no error, should never be exited */
  {
    __WFI();
  }  
  DisableADC();
  SysTick_Config(48000); /* 1ms config */
  while (1) /* Infinite loop */
  {
  }
}

/**
  * @brief  This function enables the peripheral clocks on GPIO port C,
  *         configures GPIO PC9 in output mode for the Green LED pin,
  *         configures GPIO PC8 in output mode for the orange LED pin,
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureGPIO(void)
{  
  /* (1) Enable the peripheral clock of GPIOC */
  /* (2) Select output mode (01) on GPIOC pin 8 and 9 */
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; /* (1) */  
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8|GPIO_MODER_MODER9)) \
               | (GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0); /* (2) */  
}


/**
  * @brief  This function enables the peripheral clocks on GPIO ports A,B,C
  *         configures PA1, PB1 and PC0 in Analog mode.
  *         For portability, some GPIO are again enabled.
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureGPIOforADC(void)
{
  /* (1) Enable the peripheral clock of GPIOA, GPIOB and GPIOC */
  /* (2) Select analog mode for PA1 */
  /* (3) Select analog mode for PB1 */
  /* (4) Select analog mode for PC0 */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; /* (1) */
  GPIOA->MODER |= GPIO_MODER_MODER1; /* (2) */
  GPIOB->MODER |= GPIO_MODER_MODER1; /* (3) */
  GPIOC->MODER |= GPIO_MODER_MODER0; /* (4) */ 
}


/**
  * @brief  This function enables the clock in the RCC for the ADC
  *         and set the peripheral clock (PCLK) prescaler to 2
  * @param  None
  * @retval None
  */
__INLINE void SetClockForADC(void)
{
  /* (1) Enable the peripheral clock of the ADC */
  /* (2) Start HSI14 RC oscillator */ 
  /* (3) Wait HSI14 is ready */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* (1) */
  RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
  while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
  {
    /* For robust implementation, add here time-out management */
  }  
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
  * @brief  This function configures the ADC to convert sequentially 4 channels
  *         in continuous mode.
  *         The conversion frequency is 14MHz 
  *         The interrupt on overrun is enabled and the NVIC is configured
  * @param  None
  * @retval None
  */
__INLINE void ConfigureADC(void)
{
/* defines the upper limit 15% above the factory value 
   the value is adapted according to the application power supply
   versus the factory calibration power supply */
uint16_t vrefint_high = (*VREFINT_CAL_ADDR)* VDD_CALIB / VDD_APPLI * 115 / 100; 
/* defines the lower limit 15% below the factory value 
   the value is adapted according to the application power supply
   versus the factory calibration power supply */
uint16_t vrefint_low = (*VREFINT_CAL_ADDR) * VDD_CALIB / VDD_APPLI * 85 / 100;

  /* (1) Select HSI14 by writing 00 in CKMODE (reset value) */ 
  /* (2) Select the continuous mode 
         and configure the Analog watchdog to monitor only CH17 */
  /* (3) Define analog watchdog range : 16b-MSW is the high limit 
         and 16b-LSW is the low limit */
  /* (4) Select CHSEL1, CHSEL9, CHSEL10 and CHSEL17 */
  /* (5) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than 17.1us */
  /* (6) Enable interrupts on EOC, EOSEQ and Analog Watchdog */
  /* (7) Wake-up the VREFINT (only for VBAT, Temp sensor and VRefInt) */
  //ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */   
  ADC1->CFGR1 |= ADC_CFGR1_CONT \
              | (17<<26) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL; /* (2) */
  ADC1->TR = (vrefint_high << 16) + vrefint_low; /* (3)*/
  ADC1->CHSELR = ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL9 | ADC_CHSELR_CHSEL10 | ADC_CHSELR_CHSEL17; /* (4) */
  ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* (5) */
  ADC1->IER = ADC_IER_EOCIE | ADC_IER_EOSEQIE | ADC_IER_AWDIE; /* (6) */
  ADC->CCR |= ADC_CCR_VREFEN; /* (7) */

  /* Configure NVIC for ADC */
  /* (8) Enable Interrupt on ADC */
  /* (9) Set priority for ADC */
  NVIC_EnableIRQ(ADC1_COMP_IRQn); /* (8) */
  NVIC_SetPriority(ADC1_COMP_IRQn,0); /* (9) */
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
  * @brief  This function handles ADC interrupt request.
  *         It stores the data register while EOC occurs.
  *         It reinitializes the CurrentChannel at the End of Sequence
  *         In case of Analog Watchdog, a warning is reported
  * @param  None
  * @retval None
  */
void ADC1_COMP_IRQHandler(void)
{
  if ((ADC1->ISR & (ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_AWD)) == 0)  /* Check if one the expected flag is set */
  {
    error |= ERROR_UNEXPECTED_ADC_IT; /* Report unexpected ADC interrupt occurrence */
  }
  else
  {
    if ((ADC1->ISR & ADC_ISR_AWD) != 0)  /* Check if Analog watchdog has triggered the IT */
    {
      error |= WARNING_VDD_SHIFT; /* Report a warning */
      ADC1->ISR |= ADC_ISR_AWD; /* Clear the flag */
      GPIOC->BSRR = (1<<8); /* Set orange led on PC8 */
    }
    if ((ADC1->ISR & ADC_ISR_EOC) != 0)  /* Check EOC has triggered the IT */
    {
      ADC_array[CurrentChannel] = ADC1->DR; /* Read data and clears EOC flag */
      CurrentChannel++;  /* Increment the index on ADC_array */        
    }
    if ((ADC1->ISR & ADC_ISR_EOSEQ) != 0)  /* Check EOSEQ has triggered the IT */
    {
      ADC1->ISR |= ADC_ISR_EOSEQ; /* Clear the pending bit */
      CurrentChannel = 0; /* Reinitialize the CurrentChannel */
      GPIOC->ODR ^= (1<<9); /* Toggle green led on PC9 */
    }
  }
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
