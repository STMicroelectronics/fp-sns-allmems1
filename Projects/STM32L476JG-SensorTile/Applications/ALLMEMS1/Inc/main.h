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
#include "console.h" 
#include "ALLMEMS1_config.h"

/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

/* Exported functions ------------------------------------------------------- */
extern void Error_Handler(void);
extern void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *Temp1ToSend,int16_t *Temp2ToSend);

extern void Set2GAccelerometerFullScale(void);
extern void Set4GAccelerometerFullScale(void);

extern uint32_t GetPage(uint32_t Address);
extern uint32_t GetBank(uint32_t Address);

extern unsigned char ResetAccellerometerCalibrationInMemory(void);
extern unsigned char ReCallAccellerometerCalibrationFromMemory(uint16_t dataSize, uint32_t *data);
extern unsigned char SaveAccellerometerCalibrationToMemory(uint16_t dataSize, uint32_t *data);

extern unsigned char ReCallMagnetoCalibrationFromMemory(void);
extern unsigned char SaveMagnetoCalibrationToMemory(void);

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
  extern void RTC_DataConfig(uint8_t WeekDay, uint8_t Date, uint8_t Month, uint8_t Year);
  extern void RTC_TimeConfig(uint8_t Hours, uint8_t Minutes, uint8_t Seconds);
  extern void RTC_AlarmConfig(uint8_t StepHour, uint8_t StepMin, uint8_t StepSec);

  extern void RTC_GetCurrentDateTime(void);
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

extern uint8_t BufferToWrite[256];
extern int32_t BytesToWrite;

#define ANGLE_MODE MODE_PITCH_ROLL_GRAVITY_INCLINATION

/* Exported defines and variables  ------------------------------------------------------- */

/* Code for MotionFX integration - Start Section */
/* Algorithm frequency for MotionFX library [Hz] */
#define ALGO_FREQ_FX            100U
/* Algorithm period for MotionFX [ms] */
#define ALGO_PERIOD_FX          (1000U / ALGO_FREQ_FX) 
/* 10kHz/100 For MotionFX@100Hz as defaul value */
#define DEFAULT_uhCCR1_Val      (10000U / ALGO_FREQ_FX) 
/* Code for MotionFX integration - End Section */

/* Code for MotionCP & MotionGR & MotionSD & MotionTL & MotionVC integration - Start Section */
/* Algorithm frequency for MotionCP, MotionGR, MotionSD, MotionTL and MotionVC [Hz] */
#define ALGO_FREQ_CP_GR_SD_TL_VC        50U
/* Algorithm period for MotionCP, MotionGR, MotionSD, MotionTL and MotionVC [ms] */
#define ALGO_PERIOD_CP_GR_SD_TL_VC      (1000U / ALGO_FREQ_CP_GR_SD_TL_VC)
/* 10kHz/50 as defaul value for:
  MotionCP@50Hz or 
  MotionGR@50Hz or
  MotionSD@50Hz or
  MotionTL@50Hz or
  MotionVC@50Hz */
#define DEFAULT_uhCCR2_Val      (10000U / ALGO_FREQ_CP_GR_SD_TL_VC) 
/* Code for MotionCP & MotionGR & MotionSD & MotionTL & MotionVC integration - End Section */

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

/* Update frequency for Acc/Gyro/Mag sensor [Hz] */
#define FREQ_ACC_GYRO_MAG               20U
/* Update period for Acc/Gyro/Mag [ms] */
#define ALGO_PERIOD_ACC_GYRO_MAG        (1000U / FREQ_ACC_GYRO_MAG) 
/* 10kHz/20  For Acc/Gyro/Mag@20Hz */
#define DEFAULT_uhCCR4_Val              (10000U / FREQ_ACC_GYRO_MAG)

#ifdef ALLMEMS1_MOTIONFA
/* Algorithm frequency for MotionFA library [Hz] */
#define ALGO_FREQ_FA    25U
/* Compute the prescaler value to have TIM3 counter clock equal to 2 KHz */
#define TIM_CLOCK_FA    2000U
#endif /* ALLMEMS1_MOTIONFA */

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

#endif /* __MAIN_H */


