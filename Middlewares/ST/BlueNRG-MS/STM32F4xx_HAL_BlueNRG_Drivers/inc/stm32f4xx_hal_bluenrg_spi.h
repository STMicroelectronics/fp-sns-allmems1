/**
  ******************************************************************************
  * @file    stm32f4xx_hal_bluenrg_spi.h
  * @author  MCD Application Team
  * @brief   HAL specific macros for stm32f4 SPI
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
#ifndef __STM32F4XX_HAL_LPPUART_SPI_H
#define __STM32F4XX_HAL_LPPUART_SPI_H

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
   * @brief  Enable the DMA transfer in the SPI peripheral
   *
   * @param  __HANDLE__: UART handle.
   *
   * @param  __DMAREQ__: DMA Request to enabled.
   *
   * @retval None
   */
 #define __HAL_BLUENRG_SPI_ENABLE_DMAREQ(__HANDLE__, __DMAREQ__)   ((__HANDLE__)->Instance->CR2 |= (__DMAREQ__))
 
 /**
   * @brief  Disable the DMA transfer in the SPI peripheral
   *
   * @param  __HANDLE__: SPI handle.
   *
   * @param  __DMAREQ__: DMA Request to enabled.
   *
   * @retval None
   */
 #define __HAL_BLUENRG_SPI_DISABLE_DMAREQ(__HANDLE__, __DMAREQ__)   ((__HANDLE__)->Instance->CR2 &= (~(__DMAREQ__)))
     
   /**
   * @brief  Get the Receive SPI Data peripheral address
   *
   * @param  __HANDLE__: The SPI handle
   *
   * @retval The SPI Data register address
   */
 #define __HAL_BLUENRG_SPI_GET_RX_DATA_REGISTER_ADDRESS(__HANDLE__)   ((uint32_t)&(__HANDLE__)->Instance->DR)

 /**
   * @brief  Get the Transmit SPI Data peripheral address
   *
   * @param  __HANDLE__: The SPI handle
   *
   * @retval The SPI Data register address
   */
 #define __HAL_BLUENRG_SPI_GET_TX_DATA_REGISTER_ADDRESS(__HANDLE__)   ((uint32_t)&(__HANDLE__)->Instance->DR)

/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4XX_HAL_LPPUART_SPI_H */
