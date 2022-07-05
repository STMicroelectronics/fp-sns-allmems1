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
#include "stm32l4xx_hal.h"

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
/* USER CODE BEGIN EM */
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

/* @brief  Scale factor. It is used to scale acceleration from mg to g */ 
#define FROM_MG_TO_G    0.001f
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *Temp1ToSend,int16_t *Temp2ToSend);

extern unsigned char ResetAccellerometerCalibrationInMemory(void);
extern unsigned char ReCallAccellerometerCalibrationFromMemory(uint16_t dataSize, uint32_t *data);
extern unsigned char SaveAccellerometerCalibrationToMemory(uint16_t dataSize, uint32_t *data);

extern unsigned char ReCallMagnetoCalibrationFromMemory(void);
extern unsigned char SaveMagnetoCalibrationToMemory(void);
/* Exported variables  ------------------------------------------------------- */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

#define TimCCHandle             htim1
#define TimInertialHandle       htim3
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
extern uint8_t AccEventEnabled;
extern uint8_t AudioLevelTimerEnabled;

extern uint8_t ActivityRecognitionEnabled;
extern uint8_t CarryPositionEnabled;
extern uint8_t FitnessActivitiesEnabled;
extern uint8_t GestureRecognitionEnabled;
extern uint8_t MotionIntensityEnabled;
extern uint8_t SensorFusionEnabled;
extern uint8_t ECompassEnabled;
extern uint8_t PoseEstimationEnabled;
extern uint8_t Standing_VS_SittingDeskEnabled;
extern uint8_t TiltSensingEnabled;
extern uint8_t VerticalContextEnabled;
extern uint8_t AudioSourceLocalizationTimerEnabled;

extern uint8_t TIM1_CHANNEL_1_Enabled;
extern uint8_t TIM1_CHANNEL_2_Enabled;
extern uint8_t TIM1_CHANNEL_3_Enabled;
extern uint8_t TIM1_CHANNEL_4_Enabled;

extern uint8_t NodeName[];
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ALGO_FREQ_INERTIAL 20U
#define ALGO_FREQ_FX 100U
#define DEFAULT_uhCCR4_Val (10000U / ALGO_FREQ_FA_SD)
#define DEFAULT_uhCCR2_Val (10000U / ALGO_FREQ_CP_GR_TL_VC)
#define ALGO_FREQ_CP_GR_TL_VC 50U
#define TIM_CLOCK_ENV 2000U
#define ALGO_FREQ_AUDIO_LEVEL 20U
#define ALGO_FREQ_FA_SD 25U
#define ALGO_FREQ_AR_ID_PE 16U
#define TIM_CLOCK_INERTIAL 2000U
#define DEFAULT_uhCCR1_Val (10000U / ALGO_FREQ_FX)
#define ALGO_FREQ_ENV 2U
#define DEFAULT_uhCCR3_Val (10000U / ALGO_FREQ_AR_ID_PE)
#define TIM_CLOCK_AUDIO_LEVEL 10000U
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define SPI1_EXTI_Pin_Pin GPIO_PIN_0
#define SPI1_EXTI_Pin_GPIO_Port GPIOA
#define SPI1_EXTI_Pin_EXTI_IRQn EXTI0_IRQn
#define SPI1_CS_Pin_Pin GPIO_PIN_1
#define SPI1_CS_Pin_GPIO_Port GPIOA
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
#define MEMS_ACC_INT_Pin GPIO_PIN_5
#define MEMS_ACC_INT_GPIO_Port GPIOB
#define MEMS_ACC_INT_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

#define ANGLE_MODE MODE_PITCH_ROLL_GRAVITY_INCLINATION

/************************************************/
/* Algorithms Frequency and Time Period section */
/*       used for timers initialization         */
/************************************************/

/* Algorithm frequency for MotionFX library [Hz] */
/* 10kHz/100 For MotionFX@100Hz as defaul value */
/* Algorithm period for MotionFX [ms] */
#define ALGO_PERIOD_FX	(1000U / ALGO_FREQ_FX) 

/* Algorithm frequency for MotionCP, MotionGR, MotionTL and MotionVC [Hz] */
/* 10kHz/50 as defaul value for:
  MotionCP@50Hz or 
  MotionGR@50Hz or
  MotionTL@50Hz or
  MotionVC@50Hz */
/* Algorithm period for MotionCP, MotionGR, MotionTL and MotionVC [ms] */
#define ALGO_PERIOD_CP_GR_SD_TL_VC    (1000U / ALGO_FREQ_CP_GR_SD_TL_VC)

/* Algorithm frequency for MotionAR, MotionID and MotionPE libraries [Hz] */
/* Algorithm frequency for MotionAR library [Hz] */
/* 10kHz/16 as defaul value for:
  MotionAR@16Hz
  MotionID@16Hz
  MotionPE@16Hz  */
/* Algorithm period for MotionAR, MotionID and MotionPE libraries [ms] */
#define ALGO_PERIOD_AR_ID_PE	(1000U / ALGO_FREQ_AR_ID_PE) 

/* Algorithm frequency for MotionSD and MotionFA [Hz] */
/* 10kHz/25 as defaul value for:
  MotionFA@25Hz or
  MotionSD@25Hz */
/* Algorithm period for MotionSD and MotionFA [ms] */
#define ALGO_PERIOD_FA_SD	(1000U / ALGO_FREQ_FA_SD)

/* Algorithm frequency for MotionFA library [Hz] */
/* Compute the prescaler value to have TIM3 counter clock equal to 2 KHz */
/* Update period for Acc/Gyro/Mag [ms] */
#define ALGO_PERIOD_INERTIAL    (1000U / ALGO_FREQ_INERTIAL)

/* Algorithm frequency for environmental sensor [Hz] */
/* Compute the prescaler value to have TIM4 counter clock equal to 2 KHz */
/* Update period for environmental sensor [ms] */
#define ALGO_PERIOD_ENV         (1000U / ALGO_FREQ_ENV)

/* Algorithm frequency for mic audio level [Hz] */
/* Compute the prescaler value to have TIM5 counter clock equal to 10 KHz */
/* Update period for mic audio level [ms] */
#define ALGO_PERIOD_AUDIO_LEVEL (1000U / ALGO_FREQ_AUDIO_LEVEL)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
