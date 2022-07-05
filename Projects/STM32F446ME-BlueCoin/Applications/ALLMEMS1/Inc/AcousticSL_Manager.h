/**
  ******************************************************************************
  * @file    AcousticSL_Manager.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Header for AcousticSL_Manager.c
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
#ifndef _ACOUSTICSL_MANAGER_H_
#define _ACOUSTICSL_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif
  
/* Define Acoustic Source Localization for dynamic parameters */
#define SENSITIVITY_SL_HI_THRESHOLD     50  //24
#define SENSITIVITY_SL_LOW_THRESHOLD    250  
#define ANGLE_RESOLUTION                10
  
#define ACOUSTIC_SL_NO_AUDIO_DETECTED   -100


/* Exported Functions Prototypes ---------------------------------------------*/
extern void AcousticSL_Manager_init(void);
extern void SW_Task2_Callback(void);
extern void AudioProcess_SL(void);
extern void SetConfig_SL(uint16_t newThresh);

#ifdef __cplusplus
}
#endif

#endif //_ACOUSTICSL_MANAGER_H_


