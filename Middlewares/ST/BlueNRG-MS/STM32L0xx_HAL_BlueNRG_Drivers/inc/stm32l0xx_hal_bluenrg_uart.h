/**
  ******************************************************************************
  * @file    stm32l0xx_hal_bluenrg_uart.h
  * @author  MCD Application Team
  * @brief   HAL specific macros for stm32l0 lpp uart
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
#ifndef __STM32L0XX_HAL_LPPUART_UART_H
#define __STM32L0XX_HAL_LPPUART_UART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported defines --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
 /**
   * @brief  Enable the DMA transfer in the UART peripheral
   *
   * @param  __HANDLE__: UART handle.
   *
   * @param  __DMAREQ__: DMA Request to enabled.
   *
   * @retval None
   */
 #define __HAL_LPPUART_UART_ENABLE_DMAREQ(__HANDLE__, __DMAREQ__)   ((__HANDLE__)->Instance->CR3 |= (__DMAREQ__))

 /**
   * @brief  Disable the DMA transfer in the UART peripheral
   *
   * @param  __HANDLE__: UART handle.
   *
   * @param  __DMAREQ__: DMA Request to enabled.
   *
   * @retval None
   */
 #define __HAL_LPPUART_UART_DISABLE_DMAREQ(__HANDLE__, __DMAREQ__)   ((__HANDLE__)->Instance->CR3 &= (~(__DMAREQ__)))

 /**
   * @brief  Get the Receive UART Data peripheral address
   *
   * @param  __HANDLE__: The UART handle
   *
   * @retval The UART Data register address
   */
 #define __HAL_LPPUART_UART_GET_RX_DATA_REGISTER_ADDRESS(__HANDLE__)   ((uint32_t)&(__HANDLE__)->Instance->RDR)

 /**
   * @brief  Get the Transmit UART Data peripheral address
   *
   * @param  __HANDLE__: The UART handle
   *
   * @retval The UART Data register address
   */
 #define __HAL_LPPUART_UART_GET_TX_DATA_REGISTER_ADDRESS(__HANDLE__)   ((uint32_t)&(__HANDLE__)->Instance->TDR)

/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /*__STM32L0XX_HAL_LPPUART_UART_H */
