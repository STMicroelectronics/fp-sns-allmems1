/**
  ******************************************************************************
  * @file    stm32l0xx_hal_bluenrg_gpio.c
  * @author  MCD Application Team
  * @brief   HAL specific implementation for stm32l0 GPIO
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
#include "stm32l0xx_hal.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Set the GPIO PU/PD configuration
  *
  * @param 	GPIOx : The GPIO port on which the PU/PD configuration shall apply
  *
  * @param 	GPIO_Pin_Source : The GPIO pin number on which the PU/PD configuration shall apply
  *
  * @param 	GPIOPuPd : The PU/PD configuration that shall be applied
  *
  * @retval None
  */
void HAL_LPPUART_GPIO_Set_PUPD(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin_Source, uint32_t GPIOPuPd)
{
	/**
	 * This is a temporary register to avoid any unwanted transient state when performing read/modify/write
	 * operation on the destination register
	 */
	uint32_t uwTempRegister;

	uwTempRegister = GPIOx->PUPDR;
	uwTempRegister &= ~(GPIO_PUPDR_PUPD0 << ((uint16_t)GPIO_Pin_Source * 2));
	uwTempRegister |= (((uint32_t)GPIOPuPd) << ((uint16_t)GPIO_Pin_Source * 2));
	GPIOx->PUPDR = uwTempRegister;

	return;
}

/**
  * @brief  Set the GPIO Mode configuration
  *
  * @param 	GPIOx : The GPIO port on which the PU/PD configuration shall apply
  *
  * @param 	GPIO_Pin_Source : The GPIO pin number on which the PU/PD configuration shall apply
  *
  * @param 	GPIO_Mode : The Mode configuration that shall be applied
  *
  * @retval None
  */
void HAL_LPPUART_GPIO_Set_Mode(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin_Source, uint32_t GPIO_Mode)
{
	/**
	 * This is a temporary register to avoid any unwanted transient state when performing read/modify/write
	 * operation on the destination register
	 */
	uint32_t uwTempRegister;

	uwTempRegister = GPIOx->MODER;
	uwTempRegister  &= ~(GPIO_MODER_MODE0 << (GPIO_Pin_Source * 2));
	uwTempRegister |= (((uint32_t)GPIO_Mode) << (GPIO_Pin_Source * 2));
	GPIOx->MODER = uwTempRegister;

	return;
}

/**
  * @brief  Set the GPIO Alternate mode configuration
  *
  * @param 	GPIOx : The GPIO port on which the alternate configuration shall apply
  *
  * @param 	GPIO_Pin_Source : The GPIO pin number on which the alternate configuration shall apply
  *
  * @param 	GPIO_Alternate : The Alternate configuration that shall be applied
  *
  * @retval None
  */
void HAL_LPPUART_GPIO_Set_PinAFConfig(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin_Source, uint8_t GPIO_Alternate)
{
	uint32_t temp;

	/* Configure Alternate function mapped with the current IO */
	temp = GPIOx->AFR[GPIO_Pin_Source >> 3];
	temp &= ~((uint32_t)0x0F << ((uint32_t)(GPIO_Pin_Source & (uint32_t)0x07) * 4)) ;
	temp |= ((uint32_t)(GPIO_Alternate) << (((uint32_t)GPIO_Pin_Source & (uint32_t)0x07) * 4)) ;
	GPIOx->AFR[GPIO_Pin_Source >> 3] = temp;

	return;
}
