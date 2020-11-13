/**
  ******************************************************************************
  * @file    03_GenerateSinewaveWithDMA/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the DAC in order to 
  *          generate a sinewave signal using the DMA and synchronized by 
  *          the Timer 6.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO PA4
   - DAC
   - DMA
   - TIM6
   - NVIC
   - WFI
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
    - The code performs the DAC configuration and provides data in order 
      to generate a sinus wave on the DAC output PA4 centered on VDD/2.
    - The data are first computed and stored in an array. Then the DMA
      is configured and data are automatically transfered from the array 
      to the DAC by the DMA.
    - The frequency depends on INCREMENT definition, which defines the step
      between 2 DAConversions. The greater INCREMENT, the smoother the wave 
      but the lower the frequency.
    - The Timer6 is configured to generate an external trigger
      on TRGO each us. This value is limited by the computation of the sinewave.
    - The sinewave period is 72us i.e. the frequency is ~13.9kHz.
    - The signal can be monitored with an oscilloscope on PA4.
    - In case of eror, the orange blinks coding theerror code.

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
/* following define is the step to generate the sinewave, must be a divider of 90.
   The lower INCREMENT, the lower the sinewave frequency. */
#define INCREMENT 5
#define SIN_ARRAY_SIZE 360/INCREMENT

#define TIM6_DAC_IRQn (IRQn_Type)17
/* Delay value : short one is used for the error coding, long one (~1s) in case 
   of no error or between two bursts */
#define SHORT_DELAY 100
#define LONG_DELAY 1000

/* Error codes used to make the orange led blinking */
#define ERROR_DAC_DMA_UNDERRUN 0x01
#define ERROR_DMA_XFER 0x02
#define ERROR_UNEXPECTED_DMA_IT 0x04
#define ERROR_UNEXPECTED_DAC_IT 0x08
     
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t error = 0;  //initialized at 0 and modified by the functions 
uint16_t sin_data[SIN_ARRAY_SIZE]; //table containing the data to generate the sinewave 
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void ConfigureGPIOasAnalog(void);
void ConfigureDAC(void);
void ConfigureTIM6(void);
void ConfigureDMA(void);
uint16_t ComputeSinusPolynomialApprox(uint32_t x);
uint16_t GenerateWave(void);
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

  /* Initialization of the table values for DAC signal generation */
  for (x = 0; x < SIN_ARRAY_SIZE; x++)
  {
    sin_data[x] = GenerateWave();
  }
  ConfigureGPIO();
  ConfigureGPIOasAnalog();
  ConfigureDAC();
  ConfigureDMA();
  ConfigureTIM6();
  __WFI(); /* If an eror occurs or the execution is stopped by debugger the wait mode will be exited */
  SysTick_Config(48000); /* 1ms config */
  while (1) /* Infinite loop only reach in case of error */
  {
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
__INLINE void  ConfigureGPIO(void)
{  
  /* (1) Enable the peripheral clock of GPIOC */ 
  /* (2) Reset the mode and select output mode on GPIOC pin 8 and 9 */ 
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  /* (1) */
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9))\
               | (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);  /* (2) */
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
  /* (2) Select analog mode for PA4 */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (1) */
  GPIOA->MODER |= GPIO_MODER_MODER4; /* (2) */
}


/**
  * @brief  This function enables the peripheral clocks on DAC
  *         and configures the DAC to be ready to generate a signal on DAC1_OUT
  *         synchronized by TIM6 HW trigger and loaded by a DMA transfer.
  *         The output buffer is disabled to reach the closer VDD and VSS 
  *         with DAC output.
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureDAC(void)
{
  /* (1) Enable the peripheral clock of the DAC */
  /* (2) Enable DMA transfer on DAC ch1, 
         enable interrupt on DMA underrun DAC, 
         enable the DAC ch1, enable the trigger on ch 1,
         disable the buffer on ch1, 
         and select TIM6 as trigger by keeping 000 in TSEL1 */
  RCC->APB1ENR |= RCC_APB1ENR_DACEN; /* (1) */
  DAC->CR |= DAC_CR_DMAUDRIE1 | DAC_CR_DMAEN1 | DAC_CR_BOFF1 | DAC_CR_TEN1 | DAC_CR_EN1; /* (2) */  

  /* Configure NVIC for DAC */
  /* (3) Enable Interrupt on DAC Channel1 and Channel2 */
  /* (4) Set priority for DAC Channel1 and Channel2 */
  NVIC_EnableIRQ(TIM6_DAC_IRQn); /* (3)*/
  NVIC_SetPriority(TIM6_DAC_IRQn,0); /* (4) */

  DAC->DHR12R1 = 2048; /* Initialize the DAC value to middle point */
}


/**
  * @brief  This function configures the Timer6 to generate an external trigger
  *         on TRGO each microsecond.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIM6(void)
{
  /* (1) Enable the peripheral clock of the TIM6 */ 
  /* (2) Configures MMS=010 to output a rising edge at each update event */
  /* (3) Select PCLK/2 i.e. 48MHz/2=24MHz */
  /* (4) Set one update event each 1 microsecond */
  /* (5) Enable TIM6 */
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; /* (1) */ 
  TIM6->CR2 |= TIM_CR2_MMS_1; /* (2) */
  TIM6->PSC = 1; /* (3) */
  TIM6->ARR = (uint16_t)24; /* (4) */
  TIM6->CR1 |= TIM_CR1_CEN; /* (5) */
}


/**
  * @brief  This function configures the DMA to load a value from sin_data[] 
  *         to the DAC channel 1.
  *         The load is performed at DAC request.
  *         The DMA is configured in circular mode and read from memory to peripheral. 
  *         Only the transfer error can trigger an interrupt.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureDMA(void)
{
  /* (1) Enable the peripheral clock on DMA */ 
  /* (2) Configure the peripheral data register address */
  /* (3) Configure the memory address */
  /* (4) Configure the number of DMA tranfer to be performs on DMA channel x */
  /* (5) Configure increment, size (16-bits), interrupts, transfer from memory to peripheral and circular mode */
  /* (6) Enable DMA Channel x */
  RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* (1) */
  DMA1_Channel3->CPAR = (uint32_t) (&(DAC->DHR12R1)); /* (2) */
  DMA1_Channel3->CMAR = (uint32_t)(sin_data); /* (3) */
  DMA1_Channel3->CNDTR = SIN_ARRAY_SIZE; /* (4) */
  DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 \
                      | DMA_CCR_DIR | DMA_CCR_TEIE | DMA_CCR_CIRC; /* (5) */   
  DMA1_Channel3->CCR |= DMA_CCR_EN; /* (6) */

  /* Configure NVIC for DMA */
  /* (7) Enable Interrupt on DMA Channels x */
  /* (8) Set priority for DMA Channels x */  
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn); /* (7) */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn,3); /* (8) */
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
  *         It must be between 0 and 360       
  * @retval the return value is the one translated on 12-bit i.e. between 0 and 2048.
  */
uint16_t ComputeSinusPolynomialApprox(uint32_t x)
{
float sin_p7;
float angle, angle_p;

  angle = ((float)x)*3.14/180;
  sin_p7 = angle; 
  angle_p = angle*angle*angle; /* angle_p = angle^3 */
  sin_p7 -= angle_p/6;
  angle_p = angle_p*angle*angle; /* angle_p = angle^5 */
  sin_p7 += angle_p/120;
  angle_p = angle_p*angle*angle; /* angle_p = angle^7 */
  sin_p7 -= angle_p/5040;
  sin_p7 *= 2048;
  return((uint16_t)sin_p7);   
}

/**
  * @brief  This function generates a sinusoidal wave centered on VDD/2.
  *         It runs an abscissa from 0 to 2PI.
  *         It uses the symetry from 0-PI/2 in order to get the other value :
  *         sin(PI-x) = sin(x) and sin(PI+x) = -sin(x)
  * @param  None       
  * @retval the return value is the current sinus point on 12-bit i.e. between 0 and 4095.
  */
uint16_t GenerateWave(void)
{
static uint16_t i = 0;
uint16_t data = 0;

  if (i < 90)
  {
    data = 0x800 + ComputeSinusPolynomialApprox(i);
    i+=INCREMENT;
  }
  else if (i < 180)  /* PI/2 < i < PI */
  {
    data = 0x800 + ComputeSinusPolynomialApprox(180-i);
    i+=INCREMENT;
  }
  else if (i < (180 + 90))  /* PI < i < 3PI/2 */
  {
    data = 0x800 - ComputeSinusPolynomialApprox(i-180);
    i+=INCREMENT;
  }
  else if (i < 360)  /* 3PI/2 < i < 2PI */
  {
    data = 0x800 - ComputeSinusPolynomialApprox(360-i);
    i+=INCREMENT;
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
  * @brief  This function handles DMA Channel2 and 3 interrupt request.
  *         It only manages DMA error on channel 3
  * @param  None
  * @retval None
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  
  if ((DMA1->ISR & DMA_ISR_TEIF1) != 0) /* Test if transfer error on DMA channel 1 */
  {
    error |= ERROR_DMA_XFER; /* Report an error on DMA transfer */
    DMA1->IFCR |= DMA_IFCR_CTEIF1; /* Clear the flag */
  }
  else
  {
    error |= ERROR_UNEXPECTED_DMA_IT; /* Report unexpected DMA interrupt occurrence */
  }
}


/**
  * @brief  This function handles DMA Channel1 interrupt request.
  *         It only manages DMA error
  * @param  None
  * @retval None
  */
void TIM6_DAC_IRQHandler(void)
{
  
  if ((DAC->SR & DAC_SR_DMAUDR1) != 0) /* Test if transfer error on DMA and DAC channel 1 */
  {
    error |= ERROR_DAC_DMA_UNDERRUN; /* Report an error on DMA underrun */
    DAC->SR |= DAC_SR_DMAUDR1;
  }
  else
  {
    error |= ERROR_UNEXPECTED_DAC_IT; /* Report unexpected DMA interrupt occurrence */
  }
}


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
