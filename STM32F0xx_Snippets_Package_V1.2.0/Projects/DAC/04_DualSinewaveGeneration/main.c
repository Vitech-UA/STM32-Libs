/**
  ******************************************************************************
  * @file    04_DualSinewaveGeneration/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the DAC in dual mode 
  *          in order to generate two independent signals, each channel triggered 
  *          by its own timer.
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO PA4, PA5, PC8, PC9
   - DAC
   - TIM6
   - TIM7
   - DMA
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
      to generate a sinus wave on the DAC outputs PA4 and PA5 centered on VDD/2.
    - Each sinewave has its own frequency by using TIM6 to trigger DAC channel 1
      and TIM7 to trigger DAC channel 2.
    - The data are first computed and stored in an array. Then the DMA
      is configured and data are automatically transfered from the array 
      to the DAC by the DMA.
    - The Timer 6 is configured to generate an external trigger
      on TRGO each 1us.
    - The Timer 7 is configured to generate an external trigger
      on TRGO each 2.5us.  
    - The DAC amplitude is limited to respect lower and higher DAC output 
      with buffer or not as stated in the electrical parameters of the datasheet.
    - Each sinewave is dependent on the number of step defined by INCREMENTx.
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

#define INCREMENT2 15
#define SIN2_ARRAY_SIZE 360/INCREMENT2


/* following defines are voltages expressed in millivolts */
#define VDD_APPLI 3000
#define LOWER_DAC_OUT_VOLTAGE 200
#define DAC_AMPL_MAX (uint32_t)(4095 * (VDD_APPLI-2*LOWER_DAC_OUT_VOLTAGE)/VDD_APPLI)

/* Delay value : short one is used for the error coding, long one (~1s) in case 
   of no error or between two bursts */
#define SHORT_DELAY 100
#define LONG_DELAY 1000

/* Error codes used to make the orange led blinking */
#define ERROR_DAC_DMA1_UNDERRUN 0x01
#define ERROR_DAC_DMA2_UNDERRUN 0x02
#define ERROR_DMA1_XFER 0x04
#define ERROR_DMA2_XFER 0x08
#define ERROR_UNEXPECTED_DMA_IT 0x10
#define ERROR_UNEXPECTED_DAC_IT 0x20

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t error = 0;  //initialized at 0 and modified by the functions 
uint16_t sin1_data[SIN1_ARRAY_SIZE]; //table containing the data to generate the sinewave on DAC channel 1 
uint16_t sin2_data[SIN2_ARRAY_SIZE]; //table containing the data to generate the sinewave on DAC channel 2
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void ConfigureGPIOasAnalog(void);
void ConfigureDAC(void);
void ConfigureTIM6(void);
void ConfigureTIM7(void);
void ConfigureDMA(void);
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
  uint16_t x;
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f072xb.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */

  /* initialization of the tables values for DAC signal generations */
  for (x = 0; x < SIN1_ARRAY_SIZE; x++)
  {
    sin1_data[x] = GenerateWave(INCREMENT1, 4095); /* DAC 1 has its buffer disabled */
  }
  for (x = 0; x < SIN2_ARRAY_SIZE; x++)
  {
    sin2_data[x] = GenerateWave(INCREMENT2, DAC_AMPL_MAX); /* DAC 2 has its buffer enabled */
  }
  
  ConfigureGPIO();
  ConfigureGPIOasAnalog();
  ConfigureDAC();
  ConfigureDMA();
  ConfigureTIM6();
  ConfigureTIM7();
  __WFI(); /* If an error occurs or the execution is stop by debugger, the wait mode is exited */
  SysTick_Config(48000); /* 1ms config */
  while (1) /* Infinite loop only reach in case of error */
  {    
  }
}


/**
  * @brief  This function enables the peripheral clocks on GPIO ports C,
  *         configures the Green LED pin on GPIO PC9,
  *         configures the orange LED pin on GPIO PC8.
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureGPIO(void)
{  
  /* (1) Enable the peripheral clock of GPIOC */ 
  /* (2) Reset the mode and select output mode on GPIOC pin 8 and 9 */ 
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  /* (1) */
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8|GPIO_MODER_MODER9))\
               | (GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0);  /* (2) */
}


/**
  * @brief  This function enables the peripheral clocks on GPIO port A
  *         and configures PA4 and PA5 in Analog mode.
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
  *         synchronized by TIM6 HW trigger and a signal on DAC2_OUT synchronized by 
  *         TIM7 HW trigger. 
  *         Each signal is loaded by a DMA transfer.
  *         The output buffer is disabled on channel1 only to reach the closer  
  *         VDD and VSS with DAC output but enabled on channel 2.
  *         The interrupts on DMA underrun on ch 1 and ch 2 are enabled. 
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureDAC(void)
{
  /* (1) Enable the peripheral clock of the DAC */
  /* (2) Enable DMA transfer on DAC ch1 and ch2, 
         enable interrupt on DMA underrun DAC ch1 and ch2, 
         enable the DAC ch1 and ch2,
         select TIM6 as trigger by keeping 000 in TSEL1  
         select TIM7 as trigger by writing 010 in TSEL2 */
  RCC->APB1ENR |= RCC_APB1ENR_DACEN; /* (1) */
  DAC->CR |= DAC_CR_TSEL2_1 | DAC_CR_DMAUDRIE2 | DAC_CR_DMAEN2 | DAC_CR_TEN2 | DAC_CR_EN2 \
           | DAC_CR_DMAUDRIE1 | DAC_CR_DMAEN1 | DAC_CR_BOFF1 | DAC_CR_TEN1 | DAC_CR_EN1; /* (2) */  
  /* Congigure NVIC for DAC */
  /* (3) Enable Interrupt on DAC Channel1 and Channel2 */
  /* (4) Set priority for DAC Channel1 and Channel2 */
  NVIC_EnableIRQ(TIM6_DAC_IRQn); /* (3) */
  NVIC_SetPriority(TIM6_DAC_IRQn,0); /* (4) */
  
  DAC->DHR12R1 = 2048; /* Initialize the DAC value on ch1 */
  DAC->DHR12R2 = 2048; /* Initialize the DAC value on ch2 */
}


/**
  * @brief  This function configures the Timer 6 to generate an external trigger
  *         on TRGO each microsecond.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIM6(void)
{
  /* (1) Enable the peripheral clock of the TIM6 */ 
  /* (2) Configure MMS=010 to output a rising edge at each update event */
  /* (3) Select PCLK/2 i.e. 48MHz/2=24MHz */
  /* (4) Set one update event each 1 microsecond */
  /* (5) enables TIM6 */
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; /* (1) */ 
  TIM6->CR2 |= TIM_CR2_MMS_1; /* (2) */
  TIM6->PSC = 1; /* (3) */
  TIM6->ARR = (uint16_t)24; /* (4) */
  TIM6->CR1 |= TIM_CR1_CEN; /* (5) */
}


/**
  * @brief  This function configures the Timer 7 to generate an external trigger
  *         on TRGO each 2.5 microseconds.
  * @param  None
  * @retval None
  */
__INLINE void ConfigureTIM7(void)
{
  /* (1) Enable the peripheral clock of the TIM7 */ 
  /* (2) Configure MMS=010 to output a rising edge at each update event */
  /* (3) Select PCLK/2 i.e. 48MHz/2=24MHz */
  /* (4) Set one update event each 2.5 microseconds */
  /* (5) enable TIM7 */
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; /* (1) */ 
  TIM7->CR2 |= TIM_CR2_MMS_1; /* (2) */
  TIM7->PSC = 1; /* (3) */
  TIM7->ARR = (uint16_t)60; /* (4) */
  TIM7->CR1 |= TIM_CR1_CEN; /* (5) */
}


/**
  * @brief  This function configures :
  *           - the DMA channel 3 to load a value from sin1_data[] to the DAC channel 1 
  *           - the DMA channel 4 to load a value from sin2_data[] to the DAC channel 2.
  *         The transfer is performed at DAC request.
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
  DMA1_Channel3->CMAR = (uint32_t)(sin1_data); /* (3) */
  DMA1_Channel3->CNDTR = SIN1_ARRAY_SIZE; /* (4) */
  DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 \
                      | DMA_CCR_DIR | DMA_CCR_TEIE | DMA_CCR_CIRC; /* (5) */   
  DMA1_Channel3->CCR |= DMA_CCR_EN; /* (6) */
  
  /* Configure channel 4 */
  DMA1_Channel4->CPAR = (uint32_t) (&(DAC->DHR12R2)); /* (2) */
  DMA1_Channel4->CMAR = (uint32_t)(sin2_data); /* (3) */
  DMA1_Channel4->CNDTR = SIN2_ARRAY_SIZE; /* (4) */
  DMA1_Channel4->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 \
                      | DMA_CCR_DIR | DMA_CCR_TEIE | DMA_CCR_CIRC; /* (5) */   
  DMA1_Channel4->CCR |= DMA_CCR_EN; /* (6) */
  
  /* Configure NVIC for DMA */
  /* (7) Enable Interrupt on DMA Channels 3 and 4 */
  /* (8) Set priority for DMA Channels 3 and 4 */  
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn); /* (7) */
  NVIC_SetPriority(DMA1_Channel2_3_IRQn,3); /* (8) */
  
  NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn); /* (7)*/
  NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn,0); /* (8) */
  
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

  if (i < 90)
  {
    data = 0x800 + ComputeSinusPolynomialApprox(i, amplitude);
    i+=increment;
  }
  else if (i < 180)  /* PI/2 < i < PI */
    {
    data = 0x800 + ComputeSinusPolynomialApprox(180-i, amplitude);
    i+=increment;
  }
  else if (i < (180 + 90))  /* PI < i < 3PI/2 */
  {
    data = 0x800 - ComputeSinusPolynomialApprox(i-180, amplitude);
    i+=increment;
  }
  else if (i < 360)  /* 3PI/2 < i < 2PI */
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
      long_counter = LONG_DELAY << 2;
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
    error |= ERROR_DMA1_XFER; /* Report DMA transfer error on DAC channel1 */
    DMA1->IFCR |= DMA_IFCR_CTEIF1; /* Clear the flag */
  }
  else
{
    error |= ERROR_UNEXPECTED_DMA_IT; /* Report unexpected DMA interrupt occurrence */
  }
}


/**
  * @brief  This function handles DMA Channel2 and 3 interrupt request.
  *         It only manages DMA error on channel 3
  * @param  None
  * @retval None
  */
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
  
  if ((DMA1->ISR & DMA_ISR_TEIF2) != 0) /* Test if transfer error on DMA channel 2 */
  {
    error |= ERROR_DMA2_XFER; /* Report DMA transfer error on DAC channel2 */
    DMA1->IFCR |= DMA_IFCR_CTEIF2;  /* Clear the flag */
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
    error |= ERROR_DAC_DMA1_UNDERRUN; /* Report DAC underrun error on DAC channel 1 */
    DAC->CR &= ~DAC_CR_EN1;  /* Disable DAC channel 1 */
    DAC->CR &= ~DAC_CR_DMAEN1; /* Disable DMA on DAC channel 1 */
    DAC->SR |= DAC_SR_DMAUDR1; /* Clear the flag */
  }
  else if ((DAC->SR & DAC_SR_DMAUDR2) != 0) /* Test if transfer error on DMA and DAC channel 2 */
  {
    error |= ERROR_DAC_DMA2_UNDERRUN; /* Report DAC underrun error on DAC channel 2 */
    DAC->CR &= ~DAC_CR_EN2;  /* Disable DAC channel 2 */
    DAC->CR &= ~DAC_CR_DMAEN2; /* Disable DMA on DAC channel 2 */
    DAC->SR |= DAC_SR_DMAUDR2; /* Clear the flag */
    
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
