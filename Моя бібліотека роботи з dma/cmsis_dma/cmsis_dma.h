/*
 * cmsis_dma.h
 *
 *  Created on: 15 дек. 2020 г.
 *      Author: viktor.starovit
 */

#ifndef CMSIS_DMA_H_
#define CMSIS_DMA_H_

#include <stdint.h>

#define STM32F7
//#define STM32F3
//#define STM32F1
//#define STM32F0

#ifdef STM32F7
#include "stm32f7xx.h"
#endif

#ifdef STM32F4
#include "stm32f4xx.h"
#endif

#ifdef STM32F3
#include "stm32f3xx.h"
#endif

#ifdef STM32F1
#include "stm32f1xx.h"
#endif

#ifdef STM32F0
#include "stm32f0xx.h"
#endif

#ifdef STM32F7
//0
#define DMAEnable ((uint32_t)(1<<0)) // 1: stream enabled
#define DMADisable ((uint32_t)0)     // 0: stream disabled
//1
#define DirectModeError_Int_Enable ((uint32_t)(1<<1)) //1: DME interrupt enabled
#define DirectModeError_Int_Disable ((uint32_t)0)     //0:  DME interrupt disabled
//2
#define TransError_Int_Enable ((uint32_t)(1<<2)) // 1: TE interrupt enabled
#define TransError_Int_Disable ((uint32_t)0)     // 0: TE interrupt disabled
//3
#define HalfCompl_Int_Enable ((uint32_t)1<<3) // 1: HT interrupt enabled
#define HalfCompl_Int_Disable ((uint32_t)0)   // 0: HT interrupt disabled
//4
#define TransCompl_Int_Enable ((uint32_t)(1<<4)) // 1: TC interrupt enabled
#define TransCompl_Int_Disable ((uint32_t)0)     // 0: TC interrupt disabled
//5
#define PeriphFlow_ControllerDMA ((uint32_t)(1<<5)) /* 1: The peripheral is the flow controller
                                                          This bit is protected and can be written only if EN is ‘0’.
                                                          When the memory-to-memory mode is selected (bits DIR[1:0]=10),
                                                          then this bit is automatically forced to 0 by hardware.*/

#define PeriphFlow_ControllerPeriph ((uint32_t)0)   // 0: DMA is the flow controller
//6-7
#define PeriphToMemMode ((uint32_t)(0x00 << 6))  // 00: peripheral-to-memory
#define MemToPeriphMode ((uint32_t)(0x01<<6))    // 01: memory-to-peripheral
#define MemToMemMode ((uint32_t)(0x02<<6))       // 10: memory-to-memory
//8
#define CircularMode_Enable ((uint32_t)1<<8)     // 1: circular mode enabled
#define CircularMode_Disable ((uint32_t)0)       // 0: circular mode disabled
//9
#define PeripheralInc_Enable ((uint32_t)(1<<9))  /* 1: peripheral address pointer is incremented after each data transfer (increment is done
                                                      according to PSIZE) */
#define PeripheralInc_Disable ((uint32_t)0)      // 0: peripheral address pointer is fixed
//10
#define MemoryInc_Enable ((uint32_t)(1<<10)) /* 1: memory address pointer is incremented after each data transfer (increment is done
                                                according to MSIZE) */
#define MemoryInc_Disable ((uint32_t)0)      // 0: memory address pointer is fixed
//11-12
#define PeriphDataSize_Byte ((uint32_t)(0 << 11))         // 00: byte (8-bit)
#define PeriphDataSize_HalfWord ((uint32_t)(0x01) << 11)  // 01: half-word (16-bit)
#define PeriphDataSize_Word ((uint32_t)(0x02) << 11)      // 10: word (32-bit)
//13-14
#define MemDataSize_Byte ((uint32_t)(0 << 13))         // 00: byte (8-bit)
#define MemDataSize_HalfWord ((uint32_t)(0x01) << 13)  // 01: half-word (16-bit)
#define MemDataSize_Word ((uint32_t)(0x02) << 13)      // 10: word (32-bit)
//15
#define PeriphIncrementOffsetSizePSIZE ((uint32_t)(0 << 15))  // 0: The offset size for the peripheral address calculation is linked to the PSIZE
#define PeriphIncrementOffsetSizeWord ((uint32_t)(1<< 15))   // 1: The offset size for the peripheral address calculation is fixed to 4 (32-bit alignment)
//16-17
#define DMA_Priority_Low ((uint32_t) 0<<16)       // 00: low
#define DMA_Priority_Med ((uint32_t) 1<<16)       // 01: medium
#define DMA_Priority_Hi  ((uint32_t) 2<<16)       // 10: high
#define DMA_Priority_VHi ((uint32_t) 3<<16)       // 11: very high
//18
#define DoubleBufferModeUse ((uint32_t) 1<<18)    //1: memory target switched at the end of the DMA transfer
#define DoubleBufferModeNotUse ((uint32_t) 0<<18) //0: no buffer switching at the end of transfer
//19
#define CurrentTargetMemoryIsMemory0 ((uint32_t) 0<<19) // 0: current target memory is Memory 0 (addressed by the DMA_SxM0AR pointer)
#define CurrentTargetMemoryIsMemory1 ((uint32_t) 1<<19) // 1: current target memory is Memory 1 (addressed by the DMA_SxM1AR pointer)
//20
//Reserved, must be kept at reset value.
//21-22
#define PeriphBurstTransferSingleTransfer ((uint32_t) 0<<21) // 00: single transfer
#define PeriphBurstTransferINCR4 ((uint32_t) 1<<21)  // 01: INCR4 (incremental burst of 4 beats // These bits are protected and can be written only if EN is ‘0
#define PeriphBurstTransferINCR8 ((uint32_t) 2<<21)  // 10: INCR8 (incremental burst of 8 beats) // These bits are protected and can be written only if EN is ‘0
#define PeriphBurstTransferINCR16 ((uint32_t) 3<<21) // 11: INCR16 (incremental burst of 16 beats) // These bits are protected and can be written only if EN is ‘0
/* These bits are protected and can be written only if EN is ‘0
In direct mode, these bits are forced to 0x0 by hardware */
//23-24
#define MemBurstTransferSingleTransfer ((uint32_t) 0<<23) // 00: single transfer
#define MemBurstTransferINCR4 ((uint32_t) 1<<23)  // 01: INCR4 (incremental burst of 4 beats // These bits are protected and can be written only if EN is ‘0
#define MemBurstTransferINCR8 ((uint32_t) 2<<23)  // 10: INCR8 (incremental burst of 8 beats) // These bits are protected and can be written only if EN is ‘0
#define MemBurstTransferINCR16 ((uint32_t) 3<<23) // 11: INCR16 (incremental burst of 16 beats) // These bits are protected and can be written only if EN is ‘0
/* These bits are protected and can be written only if EN is ‘0
In direct mode, these bits are forced to 0x0 by hardware */

//25-26
#define SelectCh0 ((uint32_t) 0<<25) // 000: channel 0 selected
#define SelectCh1 ((uint32_t) 1<<25) // 001: channel 1 selected
#define SelectCh2 ((uint32_t) 2<<25) // 010: channel 2 selected
#define SelectCh3 ((uint32_t) 3<<25) // 011: channel 3 selected
#define SelectCh4 ((uint32_t) 4<<25) // 100: channel 4 selected
#define SelectCh5 ((uint32_t) 5<<25) // 101: channel 5 selected
#define SelectCh6 ((uint32_t) 6<<25) // 110: channel 6 selected
#define SelectCh7 ((uint32_t) 7<<25) // 111: channel 7 selected
/*These bits are protected and can be written only if EN is ‘0’.*/

#endif

void DMA_Init(DMA_Stream_TypeDef *Channel, uint32_t Src, uint32_t Dst,
		uint32_t Size, uint32_t Conf);
void DMA_Enable(DMA_Stream_TypeDef *Channel);
void DMA_Disable(DMA_Stream_TypeDef *Channel);
uint32_t DMA_GetCurrentDataCounter(DMA_Stream_TypeDef *Channel);
void DMA_DeInit(DMA_Stream_TypeDef *Channel);
void DMA1_Stream2_IRQHandler(void);
#endif /* CMSIS_DMA_H_ */
