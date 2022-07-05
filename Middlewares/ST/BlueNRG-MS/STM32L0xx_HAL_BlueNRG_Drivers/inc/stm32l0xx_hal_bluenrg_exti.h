/**
  ******************************************************************************
  * @file    stm32l0xx_hal_bluenrg_exti.h
  * @author  MCD Application Team
  * @brief   HAL specific macros for stm32l0 EXTI
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
#ifndef __STM32L0XX_HAL_BLUENRG_EXTI_H
#define __STM32L0XX_HAL_BLUENRG_EXTI_H

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
   * @brief  Disable EXTI interrupt on selected GPIO
   *
   * @param  __EXTILINE__: specifies the GPIO Exti sources to be disabled.
   *
   * @retval None
   */
 #define __HAL_LPPUART_EXTI_DISABLE_IT(__EXTILINE__)   (EXTI->IMR &= (~(__EXTILINE__)))

 /**
   * @brief  Enable EXTI interrupt on selected GPIO
   *
   * @param  __EXTILINE__: specifies the GPIO Exti sources to be enabled.
   *
   * @retval None
   */
 #define __HAL_LPPUART_EXTI_ENABLE_IT(__EXTILINE__)   (EXTI->IMR |= (__EXTILINE__))

 /**
   * @brief  Enable EXTI rising edge configuration on selected GPIO
   *
   * @param  __EXTILINE__: specifies the GPIO Exti sources to be disabled.
   *
   * @retval None
   */
 #define __HAL_LPPUART_EXTI_ENABLE_RISING_EDGE(__EXTILINE__)   (EXTI->RTSR |= (__EXTILINE__))

 /**
   * @brief  Enable EXTI falling edge configuration on selected GPIO
   *
   * @param  __EXTILINE__: specifies the GPIO Exti sources to be disabled.
   *
   * @retval None
   */
 #define __HAL_LPPUART_EXTI_ENABLE_FALLING_EDGE(__EXTILINE__)   (EXTI->FTSR |= (__EXTILINE__))

/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /*__STM32L0XX_HAL_BLUENRG_EXTI_H */
