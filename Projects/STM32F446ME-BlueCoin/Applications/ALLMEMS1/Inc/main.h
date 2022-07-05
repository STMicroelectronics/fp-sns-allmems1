/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for main.c file.
  *          This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "hci_tl_interface.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ALLMEMS1_config.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void Error_Handler(void);
extern void ReadEnvironmentalData(int32_t *PressToSend, int16_t *Temp1ToSend);;

extern void Set2GAccelerometerFullScale(void);
extern void Set4GAccelerometerFullScale(void);

extern unsigned char ReCallCalibrationFromMemory(void);
extern unsigned char SaveCalibrationToMemory(void);

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

#define TimCCHandle             htim1
#define TimEnvHandle            htim4
#define TimAudioDataHandle      htim6
#define TimBattPlugHandle       htim7

extern uint8_t BufferToWrite[256];
extern int32_t BytesToWrite;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM_CLOCK_ENV 2000U
#define ALGO_FREQ_AR 16U
#define FREQ_ACC_GYRO_MAG 20U
#define DEFAULT_uhCCR1_Val (10000U / ALGO_FREQ_FX)
#define ALGO_FREQ_ENV 2U
#define ALGO_FREQ_FX 100U
#define DEFAULT_uhCCR4_Val (10000U / FREQ_ACC_GYRO_MAG)
#define DEFAULT_uhCCR3_Val (10000U / ALGO_FREQ_AR)
#define ALGO_FREQ_AUDIO_LEVEL 20U
#define TIM_CLOCK_AUDIO_LEVEL 10000U
#define BSP_LSM6DSM_INT2_Pin GPIO_PIN_4
#define BSP_LSM6DSM_INT2_GPIO_Port GPIOC
#define BSP_LSM6DSM_INT2_EXTI_IRQn EXTI4_IRQn
#define BSP_LSM6DSM_INT1_Pin GPIO_PIN_0
#define BSP_LSM6DSM_INT1_GPIO_Port GPIOA
#define BSP_LSM6DSM_INT1_EXTI_IRQn EXTI0_IRQn
/* USER CODE BEGIN Private defines */
/************************************************/
/* Algorithms Frequency and Time Period section */
/*       used for timers initialization         */
/************************************************/

/* Code for MotionFX integration - Start Section */
/* Algorithm frequency for MotionFX library [Hz] */
/* 10kHz/100 For MotionFX@100Hz as defaul value */
/* Algorithm period for MotionFX [ms] */
#define ALGO_PERIOD_FX	(1000U / ALGO_FREQ_FX) 
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
/* Algorithm frequency for MotionAR library [Hz] */
/* 10kHz/16  For MotionAR@16Hz as defaul value */
/* Algorithm period for MotionAR [ms] */
#define ALGO_PERIOD_AR	(1000U / ALGO_FREQ_AR) 
/* Code for MotionAR integration - End Section */

/* Update frequency for Acc/Gyro/Mag sensor [Hz] */
/* 10kHz/20  For Acc/Gyro/Mag@20Hz */
/* Update period for Acc/Gyro/Mag [ms] */
#define ALGO_PERIOD_ACC_GYRO_MAG	(1000U / FREQ_ACC_GYRO_MAG) 

/* Update frequency for environmental sensor [Hz] */
/* Compute the prescaler value to have TIM5 counter clock equal to 2 KHz */
/* Update period for environmental sensor [ms] */
#define ALGO_PERIOD_ENV (1000U / ALGO_FREQ_ENV)

/* Update frequency for mic audio level [Hz] */
/* Compute the prescaler value to have TIM6 counter clock equal to 10 KHz */
/* Update period for mic audio level [ms] */
#define ALGO_PERIOD_AUDIO_LEVEL (1000U / ALGO_FREQ_AUDIO_LEVEL)

/* Battery Features */
#define MIN_BATTERY_RANGE 2950
#define MAX_BATTERY_RANGE 4225
#define WINDOW_FILTER_VOLTAGE_VALUE_DIM 20

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
