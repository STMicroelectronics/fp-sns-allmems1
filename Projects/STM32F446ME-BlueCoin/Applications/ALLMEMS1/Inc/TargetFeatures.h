/**
  ******************************************************************************
  * @file    TargetFeatures.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Specification of the HW Features for each target platform
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
#ifndef _TARGET_FEATURES_H_
#define _TARGET_FEATURES_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
   
#include "stm32f4xx_hal.h"
#include "BlueCoin.h"
#include "BlueCoin_audio.h"
#include "BlueCoin_env_sensors.h"
#include "BlueCoin_motion_sensors.h"
#include "BlueCoin_motion_sensors_ex.h"
//#include "BlueCoin_sd.h"

   
#include "ALLMEMS1_config.h"
#include "MetaDataManager.h"

/* Code for MotionFX integration - Start Section */
#include "MotionFX_Manager.h"
#include "motion_fx.h"
/* Code for MotionFX integration - End Section */
   
/* Code for MotionAR integration - Start Section */
#include "MotionAR_Manager.h"
#include "motion_ar.h"
/* Code for MotionAR integration - End Section */

/* Code for BlueVoice integration - Start Section */
#include "AudioBV_Manager.h"
#include "bluevoice_adpcm.h"
/* Code for BlueVoice integration - End Section */

/* Code for AcousticSL integration - Start Section */
#include "AcousticSL_Manager.h"
#include "acoustic_sl.h"
/* Code for AcousticSL integration - End Section */
   
/* Code for AcousticBF integration - Start Section */
#include "AcousticBF_Manager.h"
#include "acoustic_bf.h"
/* Code for AcousticBF integration - End Section */

/* Exported defines ------------------------------------------------------- */

/* Exported macros -------------------------------------------------------- */

/* Exported types --------------------------------------------------------- */

/**
 * @brief  Target's Features data structure definition
 */
typedef struct
{
  int32_t HWAdvanceFeatures;

  uint8_t LedStatus;

  /* Code for MotionFX integration - Start Section */
  uint32_t MotionFXIsInitalized;
  /* Code for MotionFX integration - End Section */
  
  /* Code for MotionAR integration - Start Section */
  uint32_t MotionARIsInitalized;
  /* Code for MotionAR integration - End Section */
  
  /* Code for BlueVoice integration - Start Section */
  uint32_t AudioBVIsInitalized;
  /* Code for BlueVoice integration - End Section */
  
  /* Code for AcousticSL integration - Start Section */
  uint32_t AcousticSLIsInitalized;
  /* Code for AcousticSL integration - End Section */
  
  /* Code for AcousticBF integration - Start Section */
  uint32_t AcousticBFIsInitalized;
  /* Code for AcousticBF integration - End Section */
} TargetFeatures_t;

/* Exported variables ------------------------------------------------------- */
extern TargetFeatures_t TargetBoardFeatures;

/* Exported functions ------------------------------------------------------- */
extern void InitTargetPlatform(void);

extern void InitMics(uint32_t AudioFreq, uint32_t AudioVolume, uint16_t AudioInSamples);
extern void DeInitMics(void);

extern void PVD_Config(void);

extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);
extern void LedToggleTargetPlatform(void);

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */


