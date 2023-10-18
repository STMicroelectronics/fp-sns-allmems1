/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.0
  * @date    30-June-2023
  * @brief   This file contains the common defines of the application.
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
#include <math.h>
#include "ALLMEMS1_config.h"
#include "BLE_Manager.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

  /* Exported define ------------------------------------------------------------*/
/* STM32 board type*/
#define BLE_STM32_BOARD "STM32F446RE-NUCLEO"
  
/* @brief  Scale factor. It is used to scale acceleration from mg to g */ 
#define FROM_MG_TO_G    0.001f
  
/* Feature mask for SourceLocalization */
#define FEATURE_MASK_DIR_OF_ARRIVAL 0x10000000
   
/* W2ST command - SL sensitivity */
#define W2ST_COMMAND_SL_SENSITIVITY 0xCC
/* W2ST command - SL sensitivity Low */
#define W2ST_COMMAND_SL_LOW  0x00
/* W2ST command - SL sensitivity High */
#define W2ST_COMMAND_SL_HIGH  0x01

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *Temp1ToSend,int16_t *Temp2ToSend);

extern void Set2GAccelerometerFullScale(void);
extern void Set4GAccelerometerFullScale(void);

extern unsigned char ReCallMagnetoCalibrationFromMemory(void);
extern unsigned char SaveMagnetoCalibrationToMemory(void);
/* Exported variables  ------------------------------------------------------- */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

#define TimCCHandle             htim1
#define TimEnvHandle		htim4
#define TimAudioDataHandle      htim5

extern unsigned char MagnetoCalibrationDone;

extern uint32_t ForceReCalibration;

extern uint32_t uhCCR1_Val;
extern uint32_t uhCCR2_Val;
extern uint32_t uhCCR3_Val;
extern uint32_t uhCCR4_Val;

extern uint8_t EnvironmentalTimerEnabled;
extern uint8_t InertialTimerEnabled;
extern uint8_t AudioLevelTimerEnabled;
extern uint8_t AudioSourceLocalizationTimerEnabled;

extern uint8_t ActivityRecognitionEnabled;
extern uint8_t SensorFusionEnabled;
extern uint8_t ECompassEnabled;

extern uint8_t TIM1_CHANNEL_1_Enabled;
extern uint8_t TIM1_CHANNEL_3_Enabled;
extern uint8_t TIM1_CHANNEL_4_Enabled;

extern uint8_t NodeName[];
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM_CLOCK_ENV 2000U
#define ALGO_FREQ_AR 16U
#define FREQ_ACC_GYRO_MAG 20U
#define FREQ_AUDIO_MIC 20U
#define ALGO_FREQ_ENV 2U
#define ALGO_FREQ_FX 100U
#define DEFAULT_uhCCR4_Val (10000U / FREQ_ACC_GYRO_MAG)
#define DEFAULT_uhCCR3_Val (10000U / ALGO_FREQ_AR)
#define DEFAULT_uhCCR2_Val (10000U / FREQ_AUDIO_MIC)
#define DEFAULT_uhCCR1_Val (10000U / ALGO_FREQ_FX)
#define ALGO_FREQ_AUDIO_LEVEL 20U
#define TIM_CLOCK_AUDIO_LEVEL 10000U
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define SPI1_EXTI_Pin GPIO_PIN_0
#define SPI1_EXTI_GPIO_Port GPIOA
#define SPI1_EXTI_EXTI_IRQn EXTI0_IRQn
#define SPI1_CS_Pin GPIO_PIN_1
#define SPI1_CS_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SPI1_RST_Pin GPIO_PIN_8
#define SPI1_RST_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

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
/* Compute the prescaler value to have TIM4 counter clock equal to 2 KHz */
/* Update period for environmental sensor [ms] */
#define ALGO_PERIOD_ENV (1000U / ALGO_FREQ_ENV)

/* Update frequency for mic audio level [Hz] */
/* Compute the prescaler value to have TIM5 counter clock equal to 10 KHz */
/* Update period for mic audio level [ms] */
#define ALGO_PERIOD_AUDIO_LEVEL (1000U / ALGO_FREQ_AUDIO_LEVEL)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
