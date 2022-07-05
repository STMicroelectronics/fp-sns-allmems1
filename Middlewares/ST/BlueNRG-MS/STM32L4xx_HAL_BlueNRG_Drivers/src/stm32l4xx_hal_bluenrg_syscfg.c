/**
  ******************************************************************************
  * @file    stm32l4xx_hal_bluenrg_syscfg.c
  * @author  MCD Application Team
  * @brief   HAL specific implementation for stm32l4xx_hal_bluenrg_syscfg.c
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
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Selects the GPIO pin used as EXTI Line.
  *
  * @param  GPIOx : selects the GPIO port to be used as source
  *
  * @param  GPIO_Pin_Source: specifies the EXTI line to be configured.
  *
  * @retval None
  */
void HAL_LPPUART_SYSCFG_Set_EXTI_Conf(uint8_t SYSCFG_EXTI_PORTx, uint32_t GPIO_Pin_Source)
{
	uint32_t temp;

    temp = SYSCFG->EXTICR[GPIO_Pin_Source >> 2];
    temp &= ~((uint32_t)0x0F) << (4 * (GPIO_Pin_Source & 0x03));
    temp |= ((uint32_t)(SYSCFG_EXTI_PORTx) << (4 * (GPIO_Pin_Source & 0x03)));
    SYSCFG->EXTICR[GPIO_Pin_Source >> 2] = temp;

	return;
}
