/**
******************************************************************************
* @file    SensorTile_gg.h
* @author  SRA - Central Labs
* @version v2.1.6
* @date    10-Feb-2022
* @brief   This file contains definitions for SensorTile_gg.c 
*          firmware driver.
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
#ifndef __SENSORTILE_GG_H
#define __SENSORTILE_GG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "SensorTile.h"
/* Include GG sensor component driver */
#include "STC3115_Driver.h"   
	 
/** @addtogroup BSP
  * @{
  */
  
/** @addtogroup SENSORTILE
  * @{
  */ 

/** @addtogroup SENSORTILE_GG
  * @{
  */
  
/** @addtogroup SENSORTILE_GG_Exported_Types
  * @{
  */
typedef enum 
{
  GG_OK = 0,
  GG_ERROR = 1,
  GG_TIMEOUT = 2
} 
GG_StatusTypeDef;

typedef enum
{
  GG_AUTO = -1, /* Always first element and equal to -1 */
  STC3115_0,  /* GG on SensorTile Cradle */
} GG_ID_t;

/**
  * @}
  */
  
/** @addtogroup SENSORTILE_GG_Exported_Constants
  * @{
  */
#define GG_MAX_NUM     1
#define GG1            0
/**
  *@}
  */    
    

/** @addtogroup SENSORTILE_GG_Exported_Macros
  * @{
  */

/**
  * @}
  */
 
/** @addtogroup SENSORTILE_GG_Exported_Functions
  * @{
  */
/* Sensor Configuration Functions */ 
DrvStatusTypeDef BSP_GG_Init( void **handle );

DrvStatusTypeDef BSP_GG_DeInit( void **handle );
DrvStatusTypeDef BSP_GG_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GG_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GG_Get_WhoAmI( void *handle, uint8_t *who_am_i );


DrvStatusTypeDef BSP_GG_Task( void *handle, uint8_t*);
DrvStatusTypeDef BSP_GG_Reset( void *handle );
DrvStatusTypeDef BSP_GG_Stop( void *handle );
DrvStatusTypeDef BSP_GG_GetOCV( void *handle, uint32_t* );
DrvStatusTypeDef BSP_GG_GetSOC( void *handle, uint32_t* );
DrvStatusTypeDef BSP_GG_GetChargeValue( void *handle, uint32_t* );
DrvStatusTypeDef BSP_GG_GetPresence( void *handle, uint32_t* );
DrvStatusTypeDef BSP_GG_GetCurrent( void *handle, int32_t* );
DrvStatusTypeDef BSP_GG_GetTemperature( void *handle, int32_t* );
DrvStatusTypeDef BSP_GG_GetVoltage( void *handle, uint32_t* );
DrvStatusTypeDef BSP_GG_GetRemTime( void *handle, int32_t* );


uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_I2C_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_I2C_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );

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
  
#endif /* __SENSORTILE_GG_H */
  


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
