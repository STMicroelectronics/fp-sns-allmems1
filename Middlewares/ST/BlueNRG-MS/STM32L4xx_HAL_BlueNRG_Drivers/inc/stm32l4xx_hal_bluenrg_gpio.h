/**
  ******************************************************************************
  * @file    stm32l4xx_hal_bluenrg_gpio.h
  * @author  MCD Application Team
  * @brief   Header file for stm32l4xx_hal_bluenrg_gpio.c
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
#ifndef __STM32L4XX_HAL_LPPUART_GPIO_H
#define __STM32L4XX_HAL_LPPUART_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported defines --------------------------------------------------------*/
#define GPIO_PIN_POSITION_0                 ((uint8_t)0x00)
#define GPIO_PIN_POSITION_1                 ((uint8_t)0x01)
#define GPIO_PIN_POSITION_2                 ((uint8_t)0x02)
#define GPIO_PIN_POSITION_3                 ((uint8_t)0x03)
#define GPIO_PIN_POSITION_4                 ((uint8_t)0x04)
#define GPIO_PIN_POSITION_5                 ((uint8_t)0x05)
#define GPIO_PIN_POSITION_6                 ((uint8_t)0x06)
#define GPIO_PIN_POSITION_7                 ((uint8_t)0x07)
#define GPIO_PIN_POSITION_8                 ((uint8_t)0x08)
#define GPIO_PIN_POSITION_9                 ((uint8_t)0x09)
#define GPIO_PIN_POSITION_10                ((uint8_t)0x0A)
#define GPIO_PIN_POSITION_11                ((uint8_t)0x0B)
#define GPIO_PIN_POSITION_12                ((uint8_t)0x0C)
#define GPIO_PIN_POSITION_13                ((uint8_t)0x0D)
#define GPIO_PIN_POSITION_14                ((uint8_t)0x0E)
#define GPIO_PIN_POSITION_15                ((uint8_t)0x0F)

/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
 void HAL_LPPUART_GPIO_Set_PUPD(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin_Source, uint32_t GPIOPuPd);
 void HAL_LPPUART_GPIO_Set_Mode(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin_Source, uint32_t GPIO_Mode);
 void HAL_LPPUART_GPIO_Set_PinAFConfig(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin_Source, uint8_t GPIO_Alternate);


#ifdef __cplusplus
}
#endif

#endif /*__STM32L4XX_HAL_LPPUART_GPIO_H */
