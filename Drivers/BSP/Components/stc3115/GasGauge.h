/**
  ******************************************************************************
  * @file    GasGauge.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.1.0
  * @date    27-March-2023
  * @brief   This header file contains the functions prototypes for the gas gauge driver.
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
  

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __GASGAUGE_H
#define __GASGAUGE_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "component.h"

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup COMMON COMMON
 * @{
 */

/** @addtogroup GG GG
 * @{
 */

/** @addtogroup GG_Public_Types GG Public types
 * @{
 */

/** 
  * @brief  GG driver structure definition  
  */ 
typedef struct
{  
	DrvStatusTypeDef ( *Init                     ) ( DrvContextTypeDef* );
        
  DrvStatusTypeDef ( *DeInit         ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Enable  ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Sensor_Disable ) ( DrvContextTypeDef* );
  DrvStatusTypeDef ( *Get_WhoAmI     ) ( DrvContextTypeDef*, uint8_t* );
  
	DrvStatusTypeDef ( *Task                     ) ( DrvContextTypeDef*,uint8_t* );
	DrvStatusTypeDef ( *Reset                    ) ( DrvContextTypeDef* );
	DrvStatusTypeDef ( *Stop                     ) ( DrvContextTypeDef* );
	DrvStatusTypeDef ( *GetSOC                   ) ( DrvContextTypeDef*, uint32_t* );
	DrvStatusTypeDef ( *GetOCV                   ) ( DrvContextTypeDef*, uint32_t* );
	DrvStatusTypeDef ( *GetCurrent               ) ( DrvContextTypeDef*, int32_t* );
	DrvStatusTypeDef ( *GetTemperature           ) ( DrvContextTypeDef*, int32_t* );
	DrvStatusTypeDef ( *GetVoltage               ) ( DrvContextTypeDef*, uint32_t* );
	DrvStatusTypeDef ( *GetChargeValue           ) ( DrvContextTypeDef*, uint32_t* );
	DrvStatusTypeDef ( *GetPresence              ) ( DrvContextTypeDef*, uint32_t* );
    DrvStatusTypeDef ( *GetRemTime               ) ( DrvContextTypeDef*, int32_t* );
	
  DrvStatusTypeDef ( *GetAlarmStatus           ) ( DrvContextTypeDef*, uint32_t* );
	DrvStatusTypeDef ( *GetITState               ) ( DrvContextTypeDef*, uint8_t* );
	DrvStatusTypeDef ( *AlarmSetVoltageThreshold ) ( DrvContextTypeDef*, int32_t );
	DrvStatusTypeDef ( *AlarmSetSOCThreshold     ) ( DrvContextTypeDef*, int32_t );
	DrvStatusTypeDef ( *GetIT                    ) ( DrvContextTypeDef*, uint8_t* );
	DrvStatusTypeDef ( *SetIT                    ) ( DrvContextTypeDef* );
	DrvStatusTypeDef ( *StopIT                   ) ( DrvContextTypeDef* );
	DrvStatusTypeDef ( *ClearIT                  ) ( DrvContextTypeDef* ); 
} GG_Drv_t;

/**
 * @brief  GG data structure definition
 */
typedef struct
{
  void *pComponentData; /* Component specific data. */
  void *pExtData;       /* Other data. */
} GG_Data_t;

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
 
#ifdef __cplusplus
}
#endif
 
#endif /* __GASGAUGE_H */


