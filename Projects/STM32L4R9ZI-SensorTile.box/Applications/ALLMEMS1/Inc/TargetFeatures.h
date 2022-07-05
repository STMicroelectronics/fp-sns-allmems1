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
   
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_conf.h"

#include "SensorTile.box.h"
#include "SensorTile.box_env_sensors.h"
#include "SensorTile.box_env_sensors_ex.h"
#include "SensorTile.box_motion_sensors.h"
#include "SensorTile.box_motion_sensors_ex.h"
#include "SensorTile.box_audio.h"
#include "SensorTile.box_bc.h"
   
#include "ALLMEMS1_config.h"
#include "MetaDataManager.h"

#ifdef ALLMEMS1_ENABLE_PRINTF
  #include "usbd_desc.h"
  #include "usbd_cdc.h"
  #include "usbd_cdc_interface.h"
#endif /* ALLMEMS1_ENABLE_PRINTF */

/* Code for MotionAR integration - Start Section */
#include "MotionAR_Manager.h"
#include "motion_ar.h"
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
#include "MotionCP_Manager.h"
#include "motion_cp.h"
#endif /*  ALLMEMS1_MOTIONCP */
   
/* Code for MotionFA integration - Start Section */
#include "MotionFA_Manager.h"
#include "motion_fa.h"
/* Code for MotionFA integration - End Section */
   
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

/* Exported variables ------------------------------------------------------- */
#ifdef ALLMEMS1_ENABLE_PRINTF
  extern uint8_t VComBufferToWrite[];
  extern int32_t VComBytesToWrite;
#endif /* ALLMEMS1_ENABLE_PRINTF */
  
/* Exported functions ------------------------------------------------------- */
#ifdef ALLMEMS1_ENABLE_PRINTF
  extern uint32_t VCOM_read(char *buffer, uint32_t len_max);
#endif /* ALLMEMS1_ENABLE_PRINTF */

/* Exported defines ------------------------------------------------------- */
#define MAX_TEMP_SENSORS 2

/* Exported macros ------------------------------------------------------- */

/* Exported types ------------------------------------------------------- */

/**
 * @brief  Errors type Definitions
 */
typedef enum
{
  STBOX1_NO_ERROR=0,
  STBOX1_INIT_ERROR,
  STBOX1_MEMS_ERROR,
  STBOX1_AUDIO_ERROR,
  STBOX1_FATFS,
  STBOX1_OS,
} ErrorType_t;
/**
 * @brief  Target's Features data structure definition
 */
typedef struct
{
  int32_t NumTempSensors;

  uint8_t LedStatus;

  /* Code for MotionAR integration - Start Section */
  uint32_t MotionARIsInitalized;
  /* Code for MotionAR integration - End Section */

  #ifdef ALLMEMS1_MOTIONCP
  uint32_t MotionCPIsInitalized;
  #endif /*  ALLMEMS1_MOTIONCP */
  
  /* Code for MotionFA integration - Start Section */
  uint32_t MotionFAIsInitalized;
  /* Code for MotionFA integration - End Section */
  
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
} TargetFeatures_t;

/* Exported variables ------------------------------------------------------- */
extern TargetFeatures_t TargetBoardFeatures;

/* Exported functions ------------------------------------------------------- */
extern void InitTargetPlatform(void);
extern void LedInitTargetPlatform(void);
extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);
extern void LedToggleTargetPlatform(void);

extern void InitMics(uint32_t AudioFreq, uint32_t AudioVolume, uint16_t AudioInSamples);
extern void DeInitMics(void);

extern uint32_t GetPage(uint32_t Address);
extern uint32_t GetBank(uint32_t Address);

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */


