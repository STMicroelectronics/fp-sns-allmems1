/**
  ******************************************************************************
  * @file    stm32f4xx_hal_bluenrg_dma.h
  * @author  MCD Application Team
  * @brief   HAL specific macros for stm32f4 DMA
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2014 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4XX_HAL_BLUENRG_DMA_H
#define __STM32F4XX_HAL_BLUENRG_DMA_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported defines --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
 /**
   * @brief  Set the peripheral address in the DMA
   *
   * @param  __HANDLE__: DMA handle.
   *
   * @param  __ADDRESS__: Address of the peripheral.
   *
   * @retval None
   */
 #define __HAL_BLUENRG_DMA_SET_PERIPHERAL_ADDRESS(__HANDLE__, __ADDRESS__)   ((__HANDLE__)->Instance->PAR = (__ADDRESS__))

 /**
   * @brief  Set the memory address in the DMA
   *
   * @param  __HANDLE__: DMA handle.
   *
   * @param  __ADDRESS__: Address of the memory.
   *
   * @retval None
   */
 #define __HAL_BLUENRG_DMA_SET_MEMORY_ADDRESS(__HANDLE__, __ADDRESS__)   ((__HANDLE__)->Instance->M0AR = (__ADDRESS__))
   
 /**
   * @brief  Set the memory address in the DMA
   *
   * @param  __HANDLE__: DMA handle.
   *
   * @param  __ADDRESS__: Address of the memory 1.
   *
   * @retval None
   */
 #define __HAL_BLUENRG_DMA_SET_MEMORY1_ADDRESS(__HANDLE__, __ADDRESS__)   ((__HANDLE__)->Instance->M1AR = (__ADDRESS__))
   
 /**
   * @brief  Set the Counter value in the DMA
   *
   * @param  __HANDLE__: DMA handle.
   *
   * @param  __COUNTER__: Number of data to transfer.
   *
   * @retval None
   */
 #define __HAL_BLUENRG_DMA_SET_COUNTER(__HANDLE__, __COUNTER__)   ((__HANDLE__)->Instance->NDTR = (__COUNTER__))

 
 #define __HAL_BLUENRG_DMA_SET_MINC(__HANDLE__)   ((__HANDLE__)->Instance->CR |= (DMA_SxCR_MINC))
 #define __HAL_BLUENRG_DMA_CLEAR_MINC(__HANDLE__)   ((__HANDLE__)->Instance->CR &= ~(DMA_SxCR_MINC))
 
/* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif /*__STM32F4XX_HAL_BLUENRG_DMA_H */
