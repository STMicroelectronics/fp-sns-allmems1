/**
  ******************************************************************************
  * @file    stm32l4xx_it.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L4xx_IT_H
#define __STM32L4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Exported functions ------------------------------------------------------- */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* Code for MotionFA integration - Start Section */
void TIM3_IRQHandler(void);
/* Code for MotionFA integration - End Section */

void TIM4_IRQHandler(void);
void TIM5_IRQHandler(void);
void TIM1_CC_IRQHandler(void);

void AUDIO_DFSDM_DMAx_MIC1_IRQHandler(void);

void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void DFSDM1_FLT1_IRQHandler(void);

void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler( void );

#ifdef ALLMEMS1_ENABLE_PRINTF
void OTG_FS_IRQHandler(void);
#endif /* ALLMEMS1_ENABLE_PRINTF */

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4xx_IT_H */

