/**
  ******************************************************************************
  * @file    SensorTile.box_conf.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file define the SensorTile.box configuration
  *          This file should be copied to the application folder and renamed
  *          to SensorTile.box_conf.h.
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
#ifndef __SENSORTILE_BOX_CONF_H__
#define __SENSORTILE_BOX_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "SensorTile.box_bus.h"
#include "SensorTile.box_errno.h"

#define USE_MOTION_SENSOR_LIS2DW12_0        0U
#define USE_MOTION_SENSOR_LIS2MDL_0         1U
#define USE_MOTION_SENSOR_LIS3DHH_0         0U
#define USE_MOTION_SENSOR_LSM6DSOX_0        1U
#define USE_ENV_SENSOR_HTS221_0             1U
#define USE_ENV_SENSOR_LPS22HH_0            1U
#define USE_ENV_SENSOR_STTS751_0            0U
#define USE_AUDIO_IN                        1U

//set to 1 if you want to use the implemented msp, 0 to use the one in your project files
#define USE_TIM_MSP                           1
//set to 1 if you want to use the implemented irq handler, 0 to use the one in your project files
#define USE_TIM_IRQ_HANDLER                   1
//set to 1 if you want to use the implemented irq callback, 0 to use the one in your project files
#define USE_TIM_IRQ_CALLBACK                  0
  
//set to 1 if you want to use the implemented irq handler, 0 to use the one in your project files
#define USE_BC_IRQ_HANDLER                  0

//set to 1 if you want to use the implemented irq callback, 0 to use the one in your project files
#define USE_BC_IRQ_CALLBACK                 0

/* Analog and digital mics */  

/* The N_MS value defines the number of millisecond to be processed at each AudioProcess call,
that must be consistent with the N_MS_PER_INTERRUPT defined in the audio driver.
The default value of the N_MS_PER_INTERRUPT directive in the driver is set to 1, 
for backward compatibility: leaving this values as it is allows to avoid any 
modification in the application layer developed with the older versions of the driver */
#define N_MS            (N_MS_PER_INTERRUPT)
  
/* Select used microphone:
  AMIC_ONBOARD --> Only analog  mic
  DMIC_ONBOARD --> Only digital mic
  BOTH_ONBOARD --> Both analog and digital mics 
*/
#define ONBOARD_MIC             AMIC_ONBOARD
#define AUDIO_IN_CHANNELS       1

#define AUDIO_SAMPLING_FREQUENCY        16000
#define AUDIO_VOLUME_INPUT              50U
#define BSP_AUDIO_IN_IT_PRIORITY        5U

/* Define the audio peripheral used: 1U = DFSDM */ 
#define BSP_AUDIO_IN_INSTANCE           1U
  
#define PCM_AUDIO_IN_SAMPLES     AUDIO_SAMPLING_FREQUENCY/1000
  
#ifdef __cplusplus
}
#endif

#endif /* __SENSORTILE_BOX_CONF_H__*/


