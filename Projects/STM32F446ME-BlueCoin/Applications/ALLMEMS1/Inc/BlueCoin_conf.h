/**
  ******************************************************************************
  * @file    BlueCoin_conf.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.0
  * @date    30-June-2023
  * @brief   This file contains definitions for the MEMS components bus interfaces
  *          This file should be copied to the application folder and renamed
  *          to BlueCoin_conf.h.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLUECOIN_CONF_H__
#define __BLUECOIN_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
//#include "BlueCoin_bus.h"
#include "BlueCoin.h"  
#include "custom_bus.h"
#include "BlueCoin_errno.h"
  
/*Uncomment this define if you want to configure and start acquisition 
independentrly from USB functionalities*/
#define DISABLE_USB_DRIVEN_ACQUISITION
  
#define AUDIO_CHANNELS 				4
#define AUDIO_SAMPLING_FREQUENCY 		16000  
#define BSP_AUDIO_IN_INSTANCE                   0U   /* Define the audio peripheral used: 0U = I2S */  
#define BSP_AUDIO_OUT_INSTANCE                  0U   /* Define the audio peripheral used: 0U = SAI */  
#define AUDIO_VOLUME_INPUT                      64U
#define AUDIO_VOLUME_OUTPUT                     50U
#define BSP_AUDIO_IN_IT_PRIORITY                6U

#define USE_MOTION_SENSOR_LSM6DSM_0             1U
#define USE_MOTION_SENSOR_LSM303AGR_ACC_0       0U
#define USE_MOTION_SENSOR_LSM303AGR_MAG_0       1U
#define USE_ENV_SENSOR_LPS22HB_0                1U
  
#define BSP_LSM6DSM_INT1_GPIO_PORT           GPIOA
#define BSP_LSM6DSM_INT1_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define BSP_LSM6DSM_INT1_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define BSP_LSM6DSM_INT1                 GPIO_PIN_0
#define BSP_LSM6DSM_INT1_EXTI_IRQn           EXTI0_IRQn 

#define BSP_LSM6DSM_INT2_GPIO_PORT           GPIOC
#define BSP_LSM6DSM_INT2_GPIO_CLK_ENABLE()   __GPIOC_CLK_ENABLE()
#define BSP_LSM6DSM_INT2_GPIO_CLK_DISABLE()  __GPIOC_CLK_DISABLE()
#define BSP_LSM6DSM_INT2                 GPIO_PIN_4
#define BSP_LSM6DSM_INT2_EXTI_IRQn           EXTI4_IRQn 

#define BSP_LSM303AGR_INT_GPIO_PORT           GPIOB
#define BSP_LSM303AGR_INT_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define BSP_LSM303AGR_INT_GPIO_CLK_DISABLE()  __GPIOB_CLK_DISABLE()
#define BSP_LSM303AGR_INT                 GPIO_PIN_1
#define BSP_LSM303AGR_INT_EXTI_IRQn           EXTI1_IRQn 

#define BSP_LPS22HB_INT_GPIO_PORT           GPIOB
#define BSP_LPS22HB_INT_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define BSP_LPS22HB_INT_GPIO_CLK_DISABLE()  __GPIOB_CLK_DISABLE()
#define BSP_LPS22HB_INT                 GPIO_PIN_3
#define BSP_LPS22HB_INT_EXTI_IRQn           EXTI1_IRQn 

#if (AUDIO_SAMPLING_FREQUENCY == 8000)
#define MAX_DECIMATION_FACTOR 160
#else
#define MAX_DECIMATION_FACTOR 128
#endif
  
#define N_MS	(N_MS_PER_INTERRUPT)
#define PCM_AUDIO_IN_SAMPLES     AUDIO_SAMPLING_FREQUENCY/1000

#ifdef __cplusplus
}
#endif

#endif /* __BLUECOIN_CONF_H__*/


