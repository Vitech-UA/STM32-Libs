/**
  ******************************************************************************
  * @file    09_DualMode_SimultaneousSW_Trig/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the DAC in dual mode 
  *          in order to generate two signals simultaneously by software.
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO PA4 (DAC_OUT1), PA5 (DAC_OUT2)
   - DAC

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
    - The code performs the DAC configuration and provides data in order 
      to generate a sinus wave on the DAC outputs PA4 and PA5 centered on VDD/2.
    - The two sinewaves are identical and simultaneous.
    - The DAC amplitude is limited to respect lower and higher DAC output 
      without buffer as stated in the electrical parameters of the datasheet.
    - The signal can be monitored with an oscilloscope on PA4 and on PA5.
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
/* following defines are the steps to generate the sinewaves, must be a divider of 90.
   The lower INCREMENTx, the lower the DACx sinewave frequency. */
#define INCREMENT1 5
#define SIN1_ARRAY_SIZE 360/INCREMENT1

/* following defines are voltages expressed in millivolts */
#define VDD_APPLI 3000
#define LOWER_DAC_OUT_VOLTAGE 200
#define DAC_AMPL_MAX (uint32_t)(4095 * (VDD_APPLI-2*LOWER_DAC_OUT_VOLTAGE)/VDD_APPLI)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t sin1_data[SIN1_ARRAY_SIZE]; //table containing the data to generate the sinewave on DAC channel 1 
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIOasAnalog(void);
void ConfigureDAC(void);
uint16_t ComputeSinusPolynomialApprox(uint32_t x, uint16_t amplitude);
uint16_t GenerateWave(uint16_t increment, uint16_t amplitude);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  uint32_t x;
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f072xb.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */

  /* initialization of the tables values for DAC signal generations */
  for (x = 0; x < SIN1_ARRAY_SIZE; x++)
  {
    sin1_data[x] = GenerateWave(INCREMENT1, DAC_AMPL_MAX); /* DAC 1 has its buffer enabled */
  }
  
  ConfigureGPIOasAnalog();
  ConfigureDAC();
  x = 0;
  while (1) /* Infinite loop only reach in case of error */
  {    
    DAC->DHR12RD = (uint32_t)((sin1_data[x] << 16) + sin1_data[x]);
    x++;
    if (x >= SIN1_ARRAY_SIZE)
    {
      x = 0;
    }
  }
}


/**
  * @brief  This function enables the peripheral clocks on GPIO port A
  *         and configures PA4 in Analog mode.
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureGPIOasAnalog(void)
{
  /* (1) Enable the peripheral clock of GPIOA */
  /* (2) Select analog mode for PA4 and PA5 */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (1) */
  GPIOA->MODER |= GPIO_MODER_MODER4 | GPIO_MODER_MODER5; /* (2) */
}


/**
  * @brief  This function enables the peripheral clocks on DAC
  *         and configures the DAC to be ready to generate a signal on DAC1_OUT
  *         and on DAC2_OUT  as soon as  data is written in DAC_DHR12RD
  *         i.e. 12-bit left aligned data.
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureDAC(void)
{
  /* (1) Enable the peripheral clock of the DAC */
  /* (2) Enable the DAC ch1 and ch2 */
  RCC->APB1ENR |= RCC_APB1ENR_DACEN; /* (1) */
  DAC->CR |= DAC_CR_EN2 | DAC_CR_EN1; /* (2) */  
}


/**
  * @brief  This function computes the polynomial approximation of the sinus function.
  *         As this function computes for an angle between 0 and PI/2, the calling function
  *         must manage the symetry to get the correct value for the other values
  *         from PI/2 to 2PI.
  *         This function uses a polynom of 7-order
  *         P7(x) = x-x^3/3!+x^5/5!-x^7/7!
  *         This function is not optimized.
  * @param  x is an integer which corresponds to the angle in degree 
  *         x must be between 0 and 360
  *         amplitude defines the amplitude of the sinewave
  * @retval the return value is the current sinus point on 12-bit.
  */
uint16_t ComputeSinusPolynomialApprox(uint32_t x, uint16_t amplitude)
{
float sin_p7;
float angle, angle_p;

  angle = ((float)x)*314/18000;
  sin_p7 = angle; 
  angle_p = angle*angle*angle; /* angle_p = angle^3 */
  sin_p7 -= angle_p/6;
  angle_p = angle_p*angle*angle; /* angle_p = angle^5 */
  sin_p7 += angle_p/120;
  angle_p = angle_p*angle*angle; /* angle_p = angle^7 */
  sin_p7 -= angle_p/5040;
  sin_p7 *= amplitude/2;
  return((uint16_t)sin_p7);   
}


/**
  * @brief  This function generates a sinusoidal wave centered on VDD/2.
  *         It runs an abscissa from 0 to 2PI.
  *         It uses the symetry from 0-PI/2 in order to get the other value :
  *         sin(PI-x) = sin(x) and sin(PI+x) = -sin(x)
  * @param  increment defines the step i.e. accuracy and frequency of the wave
  *         amplitude defines the amplitude of the sinewave
  * @retval the return value is the current sinus point on 12-bit.
  */
uint16_t GenerateWave(uint16_t increment, uint16_t amplitude)
{
static uint16_t i = 0;
uint16_t data = 0;

  if (i<90)
  {
    data = 0x800 + ComputeSinusPolynomialApprox(i, amplitude);
    i+=increment;
  }
  else if (i<180)  /* PI/2 < i < PI */
    {
    data = 0x800 + ComputeSinusPolynomialApprox(180-i, amplitude);
    i+=increment;
  }
  else if (i < (180 + 90))  /* PI < i < 3PI/2 */
  {
    data = 0x800 - ComputeSinusPolynomialApprox(i-180, amplitude);
    i+=increment;
  }
  else if (i<(360))  /* 3PI/2 < i < 2PI */
  {
    data = 0x800 - ComputeSinusPolynomialApprox(360-i, amplitude);
    i+=increment;
    if (i >= 360)
    {
      i=0;
    }
  }
  return(data);
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
/*
void PPP_IRQHandler(void)
{
}
*/


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
