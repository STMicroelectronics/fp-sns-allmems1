/**
  ******************************************************************************
  * @file    stm32l4xx_hal_bluenrg_syscfg.h
  * @author  MCD Application Team
  * @brief   Header file for stm32l4xx_hal_bluenrg_syscfg.c
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
#ifndef __STM32L4XX_HAL_LPPUART_SYSCFG_H
#define __STM32L4XX_HAL_LPPUART_SYSCFG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported defines --------------------------------------------------------*/
#define SYSCFG_EXTI_PORTA					((uint8_t)0x00)
#define SYSCFG_EXTI_PORTB					((uint8_t)0x01)
#define SYSCFG_EXTI_PORTC					((uint8_t)0x02)
#define SYSCFG_EXTI_PORTD					((uint8_t)0x03)
#define SYSCFG_EXTI_PORTH					((uint8_t)0x05)

/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
 void HAL_LPPUART_SYSCFG_Set_EXTI_Conf(uint8_t SYSCFG_EXTI_PORTx, uint32_t GPIO_Pin_Source);

#ifdef __cplusplus
}
#endif

#endif /*__STM32L4XX_HAL_LPPUART_SYSCFG_H */
