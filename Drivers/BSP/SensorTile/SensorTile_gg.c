/**
******************************************************************************
* @file    SensorTile_gg.c
* @author  SRA - Central Labs
* @version v2.1.6
* @date    10-Feb-2022
* @brief   This file provides a set of functions needed to manage the Gas Gauge sensor.
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

/* Includes ------------------------------------------------------------------*/
#include "SensorTile_gg.h"
#include "STC3115_Driver.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup SENSORTILE
 * @{
 */

/** @addtogroup SENSORTILE_GG
 * @brief BSP functions implementation for X-NUCLEO-IKC01A1 Gas Gauge
 * @{
 */


/** @defgroup SENSORTILE_GG_Private_TypesDefinitions
 * @{
 */

/**
 * @}
 */


/** @defgroup SENSORTILE_GG_Private_Macros
 * @{
 */

/**
 * @}
 */

/** @defgroup SENSORTILE_GG_Private_Variables
 * @{
 */

static DrvContextTypeDef GG_SensorHandle[ GG_MAX_NUM ];
static GG_Data_t GG_Data[ GG_MAX_NUM ];
static STC3115_Data_t STC3115_0_Data;

/**
 * @}
 */

/** @defgroup SENSORTILE_GG_Private_FunctionPrototypes
 * @{
 */

/**
 * @}
 */

/** @defgroup SENSORTILE_GG_Exported_Functions
 * @{
 */

/**
 * @brief  Set GG Initialization.
 * @param handle pointer to the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_Init( void **handle )
{
  GG_Drv_t *driver = NULL;
  
  if(GG_SensorHandle[ STC3115_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }
  
  if ( BSP_I2C3_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Setup sensor handle. */
  GG_SensorHandle[ STC3115_0 ].who_am_i      = STC3115_ID;
  GG_SensorHandle[ STC3115_0 ].ifType        = 0; // SPI interface
  GG_SensorHandle[ STC3115_0 ].address       = STC3115_SLAVE_ADDRESS;
  GG_SensorHandle[ STC3115_0 ].spiDevice     = 0;
  GG_SensorHandle[ STC3115_0 ].instance      = STC3115_0;
  GG_SensorHandle[ STC3115_0 ].isInitialized = 0;
  GG_SensorHandle[ STC3115_0 ].isEnabled     = 0;
  GG_SensorHandle[ STC3115_0 ].isCombo       = 1;
  GG_SensorHandle[ STC3115_0 ].pData         = ( void * )&GG_Data[ STC3115_0 ];
  GG_SensorHandle[ STC3115_0 ].pVTable       = ( void * )&STC3115_Drv;
  GG_SensorHandle[ STC3115_0 ].pExtVTable    = 0;
      
  GG_Data[ STC3115_0 ].pComponentData = ( void * )&STC3115_0_Data;
  GG_Data[ STC3115_0 ].pExtData       = 0;
      
  *handle = (void *)&GG_SensorHandle[ STC3115_0 ];
//  *handle = (void *)GG_SensorHandle;
  
  driver = ( GG_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;
  
  if ( driver->Init == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->Init( (DrvContextTypeDef *)(*handle) ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize a gas gauge sensor
 * @param handle pointer to the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_DeInit( void **handle )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if ( driver->DeInit == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->DeInit( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
    
  *handle = NULL;
  
  return COMPONENT_OK;
}






/**
 * @brief Check if the gas gauge sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_IsInitialized( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *status = ctx->isInitialized;
  
  return COMPONENT_OK;
}



/**
 * @brief Check if the gas gauge sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_IsCombo( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *status = ctx->isCombo;
  
  return COMPONENT_OK;
}


/**
 * @brief Get the WHO_AM_I ID of the gas gauge sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_Get_WhoAmI( void *handle, uint8_t *who_am_i )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if ( who_am_i == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_WhoAmI( ctx, who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
 * @brief  Task to read periodically data of STC3115
 * @param handle the device handle
 * @param vm_mode the pointer to the operating mode
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_Task(void *handle, uint8_t* vm_mode)
{  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if(driver->Task == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  return driver->Task( ctx, vm_mode );
}



/**
 * @brief  Reboot memory content of GG
 * @param  handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_Reset(void *handle)
{  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if(driver->Reset == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  return driver->Reset( ctx );
}


/**
 * @brief  Stop the gas gauge system
 * @param  handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_Stop(void *handle)
{  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if(driver->Stop == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  return driver->Stop( ctx );
}


/**
 * @brief  Get the OCV
 * @param handle the device handle
 * @param ocv the pointer to the open circuit voltage value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_GetOCV(void *handle, uint32_t* ocv)
{ 
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if(driver->GetOCV == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  return driver->GetOCV( ctx, ocv);
}


/**
 * @brief  Get the SOC
 * @param handle the device handle
 * @param soc the pointer to the state of charge value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_GetSOC(void *handle, uint32_t* soc)
{ 
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if(driver->GetSOC == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  return driver->GetSOC( ctx, soc);
}


/**
 * @brief  Get the charge value
 * @param handle the device handle
 * @param chrgValue the pointer to the remaining battery charge value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_GetChargeValue(void *handle, uint32_t* chrgValue)
{ 
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if(driver->GetChargeValue == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  return driver->GetChargeValue( ctx, chrgValue);
}


/**
 * @brief  Get the presence
 * @param handle the device handle
 * @param presence the pointer to the value refers to battery presence
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_GetPresence(void *handle, uint32_t* presence)
{ 
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if(driver->GetPresence == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  return driver->GetPresence( ctx, presence);
}


/**
 * @brief  Get the current
 * @param handle the device handle
 * @param current  the pointer to the absorbed current value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_GetCurrent(void *handle, int32_t* current)
{  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if(driver->GetCurrent == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  return driver->GetCurrent( ctx, current);
}


/**
 * @brief  Get the voltage
 * @param handle the device handle
 * @param voltage  the pointer to the battery voltage value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_GetVoltage(void *handle, uint32_t* voltage)
{ 
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if(driver->GetVoltage == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  return driver->GetVoltage( ctx, voltage);
}


/**
 * @brief  Get the temperature
 * @param handle the device handle
 * @param temp  the pointer to the battery temperature value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_GetTemperature(void *handle, int32_t* temp)
{ 
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if(driver->GetTemperature == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  return driver->GetTemperature( ctx, temp);  
}

/**
 * @brief  Get the estimated remaining time
 * @param handle the device handle
 * @param remTime  the pointer to the remaining time value
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_GG_GetRemTime(void *handle, int32_t* remTime)
{  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  GG_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( GG_Drv_t * )ctx->pVTable;
  
  if(driver->GetRemTime == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  return driver->GetRemTime( ctx, remTime);
}



/** IO Write/read implemented here to maintain compatibility w.r.t old BSP **/

/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx->ifType == 0)
  {
    return BSP_I2C3_WriteReg(STC3115_SLAVE_ADDRESS, WriteAddr, pBuffer, nBytesToWrite );
  }
  
  return COMPONENT_ERROR;
}


/**
 * @brief  Reads from the sensor to a buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx->ifType == 0)
  {
    return BSP_I2C3_ReadReg(STC3115_SLAVE_ADDRESS, ReadAddr, pBuffer, nBytesToRead );
  }
  
  return COMPONENT_ERROR;
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     
