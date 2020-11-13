/**
  ******************************************************************************
  * @file    01_ProgrammingTheIWDG/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   This code example shows how to configure the IWDG
  *          in order to have a reset if not refresh.
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO  PA0, PC8, PC9,
   - SysTick
   - IWDG
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
   - Power up the board, the green LED is blinking
   - the IWDG will be refresh until the user button is pressed
   - then the orange LED is lit for a while (means a IWDG reset occurs)
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
#include "string.h"

/** @addtogroup STM32F0_Snippets
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TEMPO_ORANGE_LED  (2000) /* 2s */
#define TEMPO_GREEN_LED   (500)  /* 500ms */
#define TEMPO_REFRESH     (80)   /* 80ms, this value must be less than 100 to have a proper IWDG refresh */
#define IWDG_REFRESH      (uint32_t)(0x0000AAAA)
#define IWDG_WRITE_ACCESS (uint32_t)(0x00005555)
#define IWDG_START        (uint32_t)(0x0000CCCC)
#define IWDG_RELOAD       (500)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void Configure_GPIO_LED(void);
void Configure_IWDG(void);
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
  Configure_IWDG();
  Configure_GPIO_Button();
  Configure_EXTI();
  SysTick_Config(48000); /* 1ms config */
  
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
  * @brief  This function configures IWDG.
  * @param  None
  * @retval None
  */
__INLINE void Configure_IWDG(void)
{
   /* Enable the peripheral clock RTC */
  /* (1) Enable the LSI */
  /* (2) Wait while it is not ready */
  RCC->CSR |= RCC_CSR_LSION; /* (1) */
  while((RCC->CSR & RCC_CSR_LSIRDY) != RCC_CSR_LSIRDY) /* (2) */
  { 
    /* add time out here for a robust application */
  }

  /* Enable the peripheral clock of DBG register (uncomment for debug purpose) */
//  RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
//  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP; /* To be able to debug */
  
  /* Configure IWDG */
  /* (1) Activate IWDG (not needed if done in option bytes) */
  /* (2) Enable write access to IWDG registers */
  /* (3) Set prescaler by 8 */
  /* (4) Set reload value to have a rollover each 100ms */
  /* (5) Check if flags are reset */
  /* (6) Refresh counter */
  IWDG->KR = IWDG_START; /* (1) */
  IWDG->KR = IWDG_WRITE_ACCESS; /* (2) */
  IWDG->PR = IWDG_PR_PR_0; /* (3) */
  IWDG->RLR = IWDG_RELOAD; /* (4) */
  while(IWDG->SR) /* (5) */
  { 
    /* add time out here for a robust application */
  }
  IWDG->KR = IWDG_REFRESH; /* (6) */
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
  static uint32_t TempoOrangeLed = TEMPO_ORANGE_LED; /* 2s */
  static uint32_t TempoGreenLed = 0;
  static uint32_t TempoRefresh = 0;
  
  /* check if IWDG reset occurs */
  if((RCC->CSR & RCC_CSR_IWDGRSTF) == RCC_CSR_IWDGRSTF)
  {
    if(TempoOrangeLed)
    {
      TempoOrangeLed--;
      GPIOC->BSRR = GPIO_BSRR_BS_8;
    }
    else
    {
      RCC->CSR |= RCC_CSR_RMVF; /* Remove reset flags */
      GPIOC->BSRR = GPIO_BSRR_BR_8;
    }
  }
  else
  {
    if(TempoGreenLed++ == 0)
    {
      GPIOC->ODR ^= GPIO_ODR_9;
    }
    TempoGreenLed %= TEMPO_GREEN_LED;
  }
  
  TempoRefresh++;
  if(TempoRefresh == TEMPO_REFRESH)
  {
    TempoRefresh = 0;
    IWDG->KR = IWDG_REFRESH; /* Refresh IWDG */
  }
}


/**
  * @brief  This function handles EXTI 0 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_1_IRQHandler(void)
{
  EXTI->PR |= EXTI_PR_PR0;
  
  while(1);/* the IWDG won't be refresh */
}


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
