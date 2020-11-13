/**
  @page STM32F0xx Snippets
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    readme.txt 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    19-June-2015
  * @brief   Description of the snippet purpose
  ******************************************************************************
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
   @endverbatim

@par Snippet Description 

The snippets are code examples. 
Their main purpose is to show how to use the STM32F0xx peripherals.
They have been designed to run on the STM32F072B Discovery board (MB1076).
The target is :
    - to illustrate the Reference Manual with code,
    - to help the customer developing its own directory or writing direct register
      access code.
These examples assume the clock has been yet configured at 48MHz and the registers are in 
their reset states.
Some parts are given as example, typically the error management and must be configured 
following customer application needs.
The error management of the polling loops are kept empty and must be filled according to
customer robustness policy.
Some functions, such as sine wave generation in DAC examples, are not optimized and have been 
written only to visualize the product feature.


@par Directory contents 

Not supported peripherals:
- USB
- CRC
- CRS
- COMP

Partially supported peripherals (used but no specific snippets):
- PWR
- SYSCFG
  
@note The "system_stm32f0xx.c" can be easily customized to meet user application requirements. </a>

         

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
