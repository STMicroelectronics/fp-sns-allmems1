/**
  ******************************************************************************
  * @file    AcousticBF_Manager.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for AcousticBF_Manager.c
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
#ifndef _ACOUSTICBF_MANAGER_H_
#define _ACOUSTICBF_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Define Acoustic Beam Forming for static parameters */
#define AUDIO_SAMPLING_FREQUENCY_BF   2048
  
#define OUTPUT_STREAM_NUMBER_CHANNELS   2
  
/* Define Acoustic Beam Forming for for dynamic parameters */
#define GAIN_APPLIED_TO_SECOND_MICROPHONE       (0.0f)
#define AUDIO_BF_VOLUME_VALUE                   20

#define  RANGE_INF_MIC_M4_EXT_B 0
#define  RANGE_SUP_MIC_M4_EXT_B 80
 
#define  RANGE_INF_MIC_M1_EXT_B 100
#define  RANGE_SUP_MIC_M1_EXT_B 180



/* Exported Functions Prototypes ---------------------------------------------*/
extern void AcousticBF_Manager_init(void);
extern void SW_Task1_Callback(void);
extern void AudioProcess_BF(int32_t dir);
extern void BeamFormingSetType(uint32_t type);

extern void BeamFormingDirectionSetup(int32_t Direction);

#ifdef __cplusplus
}
#endif

#endif //_ACOUSTICBF_MANAGER_H_


