/**
  ******************************************************************************
  * @file    main.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for main.c module
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "hci.h"
#include "ALLMEMS1_config.h"
#include "BLE_Manager.h"

/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

/* Exported functions ------------------------------------------------------- */
extern void Error_Handler(void);
extern void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *Temp1ToSend,int16_t *Temp2ToSend);

extern void Set2GAccelerometerFullScale(void);
extern void Set4GAccelerometerFullScale(void);

extern unsigned char ResetAccellerometerCalibrationInMemory(void);
extern unsigned char ReCallAccellerometerCalibrationFromMemory(uint16_t dataSize, uint32_t *data);
extern unsigned char SaveAccellerometerCalibrationToMemory(uint16_t dataSize, uint32_t *data);

extern unsigned char ReCallMagnetoCalibrationFromMemory(void);
extern unsigned char SaveMagnetoCalibrationToMemory(void);
  
extern uint8_t getBlueNRG2_Version(uint8_t *hwVersion, uint16_t *fwVersion);

/* Exported variables  ------------------------------------------------------- */
extern TIM_HandleTypeDef TimInertialHandle;
extern TIM_HandleTypeDef TimEnvHandle;
extern TIM_HandleTypeDef TimAudioDataHandle;
extern TIM_HandleTypeDef TimCCHandle;

extern uint32_t uhCCR1_Val;
extern uint32_t uhCCR2_Val;
extern uint32_t uhCCR3_Val;
extern uint32_t uhCCR4_Val;

extern uint8_t EnvironmentalTimerEnabled;
extern uint8_t BatteryTimerEnabled;
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

extern uint8_t TIM1_CHANNEL_1_Enabled;
extern uint8_t TIM1_CHANNEL_2_Enabled;
extern uint8_t TIM1_CHANNEL_3_Enabled;
extern uint8_t TIM1_CHANNEL_4_Enabled;

extern uint8_t NodeName[];

extern unsigned char MagnetoCalibrationDone;
extern uint32_t ForceReCalibration;

/* Exported defines and variables  ------------------------------------------------------- */

/* Code for MotionFX integration - Start Section */
/* Algorithm frequency for MotionFX library [Hz] */
#define ALGO_FREQ_FX            100U
/* Algorithm period for MotionFX [ms] */
#define ALGO_PERIOD_FX          (1000U / ALGO_FREQ_FX) 
/* 10kHz/100 For MotionFX@100Hz as defaul value */
#define DEFAULT_uhCCR1_Val      (10000U / ALGO_FREQ_FX) 
/* Code for MotionFX integration - End Section */

/* Code for MotionCP & MotionGR & MotionTL & MotionVC integration - Start Section */
/* Algorithm frequency for MotionCP, MotionGR, MotionTL and MotionVC [Hz] */
#define ALGO_FREQ_CP_GR_TL_VC        50U
/* Algorithm period for MotionCP, MotionGR, MotionTL and MotionVC [ms] */
#define ALGO_PERIOD_CP_GR_TL_VC      (1000U / ALGO_FREQ_CP_GR_TL_VC)
/* 10kHz/50 as defaul value for:
  MotionCP@50Hz or 
  MotionGR@50Hz or
  MotionTL@50Hz or
  MotionVC@50Hz */
#define DEFAULT_uhCCR2_Val      (10000U / ALGO_FREQ_CP_GR_TL_VC) 
/* Code for MotionCP & MotionGR & MotionTL & MotionVC integration - End Section */

/* Code for MotionAR & MotionID & MotionPE integration - Start Section */
/* Algorithm frequency for MotionAR, MotionID and MotionPE libraries [Hz] */
#define ALGO_FREQ_AR_ID_PE      16U
/* Algorithm period for MotionAR, MotionID and MotionPE libraries [ms] */
#define ALGO_PERIOD_AR_ID_PE    (1000U / ALGO_FREQ_AR_ID_PE) 
/* 10kHz/16 as defaul value for:
  MotionAR@16Hz
  MotionID@16Hz
  MotionPE@16Hz  */
#define DEFAULT_uhCCR3_Val      (10000U / ALGO_FREQ_AR_ID_PE)
/* Code for MotionAR & MotionID & MotionPE integration - End Section */

/* Algorithm frequency for MotionSD and MotionFA [Hz] */
#define ALGO_FREQ_FA_SD         25U
/* Algorithm period for MotionSD and MotionFA [ms] */
#define ALGO_PERIOD_FA_SD       (1000U / ALGO_FREQ_FA_SD)
/* 10kHz/25 as defaul value for:
  MotionFA@25Hz or
  MotionSD@25Hz */
#define DEFAULT_uhCCR4_Val      (10000U / ALGO_FREQ_FA_SD)

/* Update frequency for Acc/Gyro/Mag sensor [Hz] */
#define ALGO_FREQ_INERTIAL 20U
/* Compute the prescaler value to have TIM3 counter clock equal to 2 KHz */
#define TIM_CLOCK_INERTIAL 2000U
/* Update period for Acc/Gyro/Mag [ms] */
#define ALGO_PERIOD_INERTIAL    (1000U / ALGO_FREQ_INERTIAL)

/* Update frequency for environmental sensor [Hz] */
#define ALGO_FREQ_ENV   2U
/* Compute the prescaler value to have TIM4 counter clock equal to 2 KHz */
#define TIM_CLOCK_ENV   2000U
/* Update period for environmental sensor [ms] */
#define ALGO_PERIOD_ENV (1000U / ALGO_FREQ_ENV)

/* Update frequency for mic audio level [Hz] */
#define ALGO_FREQ_AUDIO_LEVEL   20U
/* Compute the prescaler value to have TIM5 counter clock equal to 10 KHz */
#define TIM_CLOCK_AUDIO_LEVEL   10000U
/* Update period for mic audio level [ms] */
#define ALGO_PERIOD_AUDIO_LEVEL (1000U / ALGO_FREQ_AUDIO_LEVEL)

/* Range time without connect before Shut Down Mode starts (20 sec) */
#define RANGE_TIME_WITHOUT_CONNECTED  20000
#define ANGLE_MODE MODE_PITCH_ROLL_GRAVITY_INCLINATION

/* @brief  Scale factor. It is used to scale acceleration from mg to g */ 
#define FROM_MG_TO_G    0.001f

#endif /* __MAIN_H */

