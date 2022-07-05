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
#include <stdio.h>
#include <stdlib.h>
   
#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo.h"
#include "stm32l4xx_hal_conf.h"
#include "cca02m2_audio_Patch.h"

#include "iks01a3_motion_sensors.h"
#include "iks01a3_motion_sensors_ex.h"
#include "iks01a3_env_sensors.h"
#include "iks01a3_env_sensors_ex.h"

#include "ALLMEMS1_config.h"
#include "MetaDataManager.h"

/* Code for MotionAR integration - Start Section */
#include "MotionAR_Manager.h"
#include "motion_ar.h"
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
#include "MotionCP_Manager.h"
#include "motion_cp.h"
#endif /*  ALLMEMS1_MOTIONCP */
   
#ifdef ALLMEMS1_MOTIONFA
#include "MotionFA_Manager.h"
#include "motion_fa.h"
#endif /* ALLMEMS1_MOTIONFA */
   
/* Code for MotionFX integration - Start Section */
#include "MotionFX_Manager.h"
#include "motion_fx.h"
/* Code for MotionFX integration - End Section */

/* Code for MotionGR integration - Start Section */
#include "MotionGR_Manager.h"
#include "motion_gr.h"
/* Code for MotionGR integration - End Section */
   
/* Code for MotionID integration - Start Section */
#include "MotionID_Manager.h"
#include "motion_id.h"
/* Code for MotionID integration - End Section */
   
/* Code for MotionPE integration - Start Section */
#include "MotionPE_Manager.h"
#include "motion_pe.h"
/* Code for MotionPE integration - End Section */
   
/* Code for MotionSD integration - Start Section */
#include "MotionSD_Manager.h"
#include "motion_sd.h"
/* Code for MotionSD integration - End Section */
   
/* Code for MotionTL integration - Start Section */
#include "MotionTL_Manager.h"
#include "motion_tl.h"
/* Code for MotionTL integration - End Section */
   
/* Code for MotionVC integration - Start Section */
#include "MotionVC_Manager.h"
#include "motion_vc.h"
/* Code for MotionVC integration - End Section */

/* Code for AcousticSL integration - Start Section */
#include "AcousticSL_Manager.h"
#include "acoustic_sl.h"
/* Code for AcousticSL integration - End Section */

/* Exported defines ------------------------------------------------------- */
#define MAX_TEMP_SENSORS 2
   
/* Mems Board Type */
#define _IKS01A2 0
#define _IKS01A3 1

/* Exported macros ------------------------------------------------------- */

/* Exported types ------------------------------------------------------- */

/**
 * @brief  Target's Features data structure definition
 */
typedef struct
{
  int32_t NumTempSensors;

  uint8_t LedStatus;
  uint8_t mems_expansion_board;
  
  /* Code for MotionAR integration - Start Section */
  uint32_t MotionARIsInitalized;
  /* Code for MotionAR integration - End Section */

  #ifdef ALLMEMS1_MOTIONCP
  uint32_t MotionCPIsInitalized;
  #endif /*  ALLMEMS1_MOTIONCP */
  
  #ifdef ALLMEMS1_MOTIONFA
  uint32_t MotionFAIsInitalized;
  #endif /* ALLMEMS1_MOTIONFA */
  
  /* Code for MotionFX integration - Start Section */
  uint32_t MotionFXIsInitalized;
  /* Code for MotionFX integration - End Section */
  
  /* Code for MotionGR integration - Start Section */
  uint32_t MotionGRIsInitalized;
  /* Code for MotionGR integration - End Section */
  
  /* Code for MotionID integration - Start Section */
  uint32_t MotionIDIsInitalized;
  /* Code for MotionID integration - End Section */
  
  /* Code for MotionPE integration - Start Section */
  uint32_t MotionPEIsInitalized;
  /* Code for MotionPE integration - End Section */

  /* Code for MotionSD integration - Start Section */
  uint32_t MotionSDIsInitalized;
  /* Code for MotionSD integration - End Section */
  
  /* Code for MotionTL integration - Start Section */
  uint32_t MotionTLIsInitalized;
  /* Code for MotionTL integration - End Section */

  /* Code for MotionVC integration - Start Section */
  uint32_t MotionVCIsInitalized;
  /* Code for MotionVC integration - End Section */
  
  /* Code for AcousticSL integration - Start Section */
  uint32_t AcousticSLIsInitalized;
  /* Code for AcousticSL integration - End Section */
} TargetFeatures_t;

/* Exported variables ------------------------------------------------------- */
extern TargetFeatures_t TargetBoardFeatures;

/* Exported functions ------------------------------------------------------- */
extern void InitTargetPlatform(void);

extern void InitMics(uint32_t AudioFreq, uint32_t AudioVolume, uint16_t AudioInSamples);
extern void DeInitMics(void);

extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);
extern void LedToggleTargetPlatform(void);

extern uint32_t GetPage(uint32_t Address);
extern uint32_t GetBank(uint32_t Address);

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */

