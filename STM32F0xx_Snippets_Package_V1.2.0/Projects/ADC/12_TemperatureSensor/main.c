/**
  ******************************************************************************
  * @file    12_TemperatureSensor/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the ADC to compute the 
  *          junction temperature
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - ADC on TempSensor(CH16)
   - HSI 14 MHz for ADC
   - GPIO PC8 and PC9
   - TIM15

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
    - The code performs an ADC calibration then configure the ADC to 
      convert channel 16 i.e. the temperature sensor.
    - The Timer15 is configured to generate an external trigger
      on TRGO each 1s.
    - The conversion is launched by TIM15 trigger. The loop checks the end of conversion,
      compute the temperature and toggles the green led.
    - While the temperature is above 45°C, the green led continues to toggle 
      but the orange led is switched on.
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
/* Temperature sensor calibration value address */
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7C2))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
#define VDD_CALIB ((uint16_t) (330))
#define VDD_APPLI ((uint16_t) (300))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int32_t temperature_C; //contains the computed temperature
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void SetClockForADC(void);
void CalibrateADC(void);
void ConfigureADC(void);
void EnableADC(void);
void DisableADC(void);
void ConfigureTIM15(void);
int32_t ComputeTemperature(uint32_t measure);
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
  EnableADC();
  ConfigureADC();
  ConfigureTIM15();
  ADC1->CR |= ADC_CR_ADSTART; /* Start the ADC conversion */
  
  while (1) /* Infinite loop */
  {   
    while ((ADC1->ISR & ADC_ISR_EOC) == 0); /* Wait end of conversion */
    temperature_C = ComputeTemperature((int32_t) ADC1->DR);
	
    if (temperature_C > 45)
    {
      /* Report by switching on the orange led */
      GPIOC->BSRR = (1<<8); /* Set orange led on PC8 */
    }
    else
    {
      /* Stop to report the warning by switching off the orange led */
      GPIOC->BRR = (1<<8); //Switch off orange led on PC8
    }
    GPIOC->ODR ^= (1<<9); //Toggle green led on PC9
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
  * @brief  This function enables the clock in the RCC for the ADC
  *         and start HSI 14MHz dedicated RC oscillator
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
  * @brief  This function configure the ADC to convert the temperature sensor
  *         The conversion frequency is 14MHz 
  * @param  None
  * @retval None
  */
__INLINE void ConfigureADC(void)
{
  /* (1) Select HSI14 by writing 00 in CKMODE (reset value) */ 
  /* (2) Select the external trigger on TIM15_TRGO and falling edge */
  /* (3) Select CHSEL16 for temperature sensor */
  /* (4) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than 17.1us */
  /* (5) Wake-up the Temperature sensor (only for VBAT, Temp sensor and VRefInt) */
  //ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */   
  ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0 | ADC_CFGR1_EXTSEL_2; /* (2) */
  ADC1->CHSELR = ADC_CHSELR_CHSEL16; /* (3) */
  ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* (4) */
  ADC->CCR |= ADC_CCR_TSEN; /* (5) */
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

/**
  * @brief  This function computes the temperature from the temperature sensor measure
  *         The computation uses the following formula :
  *         Temp = (Vsense - V30)/Avg_Slope + 30
  *         Avg_Slope = (V110 - V30) / (110 - 30)
  *         V30 = Vsense @30°C (calibrated in factory @3.3V)
  *         V110 = Vsense @110°C (calibrated in factory @3.3V)
  *         VDD_APPLI/VDD_CALIB coefficient allows to adapt the measured value
  *         according to the application board power supply versus the 
  *         factory calibration power supply.
  * @param  measure is the a voltage measured from the temperature sensor (can have been filtered)
  * @retval returns the computed temperature
  */
int32_t ComputeTemperature(uint32_t measure)
{
  int32_t temperature;
  
  temperature = ((measure * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR ) ;	
  temperature = temperature * (int32_t)(110 - 30);                      
  temperature = temperature / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);                 
  temperature = temperature + 30;
  return(temperature);
}


/**
  * @brief  This function configures the Timer15 to generate an external trigger
  *         on TRGO each 1s.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIM15(void)
{
  /* (1) Enable the peripheral clock of the TIM15 */ 
  /* (2) Configure MMS=010 to output a rising edge at each update event */
  /* (3) Select PCLK/960 i.e. 24MHz/960=25kHz */
  /* (4) Set one update event each second */
  /* (5) Enable TIM15 */
  RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; /* (1) */ 
  TIM15->CR2 |= TIM_CR2_MMS_1; /* (2) */
  TIM15->PSC = 959; /* (3) */
  TIM15->ARR = (uint16_t)50000; /* (4) */
  TIM15->CR1 |= TIM_CR1_CEN; /* (5) */
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
