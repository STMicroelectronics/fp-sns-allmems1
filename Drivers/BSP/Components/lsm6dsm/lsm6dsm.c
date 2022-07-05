/**
 ******************************************************************************
 * @file    lsm6dsm.c
 * @author  SRA - Central Labs
 * @brief   LSM6DSM driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "lsm6dsm.h"

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @defgroup LSM6DSM LSM6DSM
 * @{
 */

/** @defgroup LSM6DSM_Exported_Variables LSM6DSM Exported Variables
 * @{
 */

LSM6DSM_CommonDrv_t LSM6DSM_COMMON_Driver =
{
  LSM6DSM_Init,
  LSM6DSM_DeInit,
  LSM6DSM_ReadID,
  LSM6DSM_GetCapabilities,
};

LSM6DSM_ACC_Drv_t LSM6DSM_ACC_Driver =
{
  LSM6DSM_ACC_Enable,
  LSM6DSM_ACC_Disable,
  LSM6DSM_ACC_GetSensitivity,
  LSM6DSM_ACC_GetOutputDataRate,
  LSM6DSM_ACC_SetOutputDataRate,
  LSM6DSM_ACC_GetFullScale,
  LSM6DSM_ACC_SetFullScale,
  LSM6DSM_ACC_GetAxes,
  LSM6DSM_ACC_GetAxesRaw,
};

LSM6DSM_GYRO_Drv_t LSM6DSM_GYRO_Driver =
{
  LSM6DSM_GYRO_Enable,
  LSM6DSM_GYRO_Disable,
  LSM6DSM_GYRO_GetSensitivity,
  LSM6DSM_GYRO_GetOutputDataRate,
  LSM6DSM_GYRO_SetOutputDataRate,
  LSM6DSM_GYRO_GetFullScale,
  LSM6DSM_GYRO_SetFullScale,
  LSM6DSM_GYRO_GetAxes,
  LSM6DSM_GYRO_GetAxesRaw,
};

/**
 * @}
 */

/** @defgroup LSM6DSM_Private_Function_Prototypes LSM6DSM Private Function Prototypes
 * @{
 */

static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t LSM6DSM_ACC_SetOutputDataRate_When_Enabled(LSM6DSM_Object_t *pObj, float Odr);
static int32_t LSM6DSM_ACC_SetOutputDataRate_When_Disabled(LSM6DSM_Object_t *pObj, float Odr);
static int32_t LSM6DSM_GYRO_SetOutputDataRate_When_Enabled(LSM6DSM_Object_t *pObj, float Odr);
static int32_t LSM6DSM_GYRO_SetOutputDataRate_When_Disabled(LSM6DSM_Object_t *pObj, float Odr);

/**
 * @}
 */

/** @defgroup LSM6DSM_Exported_Functions LSM6DSM Exported Functions
 * @{
 */

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_RegisterBusIO(LSM6DSM_Object_t *pObj, LSM6DSM_IO_t *pIO)
{
  int32_t ret = LSM6DSM_OK;

  if (pObj == NULL)
  {
    ret = LSM6DSM_ERROR;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.BusType   = pIO->BusType;
    pObj->IO.Address   = pIO->Address;
    pObj->IO.WriteReg  = pIO->WriteReg;
    pObj->IO.ReadReg   = pIO->ReadReg;
    pObj->IO.GetTick   = pIO->GetTick;

    pObj->Ctx.read_reg  = ReadRegWrap;
    pObj->Ctx.write_reg = WriteRegWrap;
    pObj->Ctx.handle   = pObj;

    if (pObj->IO.Init == NULL)
    {
      ret = LSM6DSM_ERROR;
    }
    else if (pObj->IO.Init() != LSM6DSM_OK)
    {
      ret = LSM6DSM_ERROR;
    }
    else
    {
      if (pObj->IO.BusType == LSM6DSM_SPI_3WIRES_BUS) /* SPI 3-Wires */
      {
        /* Enable the SPI 3-Wires support only the first time */
        if (pObj->is_initialized == 0U)
        {
          /* Enable SPI 3-Wires on the component */
          uint8_t data = 0x0C;

          if (LSM6DSM_Write_Reg(pObj, LSM6DSM_CTRL3_C, data) != LSM6DSM_OK)
          {
            ret = LSM6DSM_ERROR;
          }
        }
      }
    }
  }

  return ret;
}

/**
 * @brief  Initialize the LSM6DSM sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_Init(LSM6DSM_Object_t *pObj)
{
  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. */
  if (lsm6dsm_auto_increment_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable BDU */
  if (lsm6dsm_block_data_update_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* FIFO mode selection */
  if (lsm6dsm_fifo_mode_set(&(pObj->Ctx), LSM6DSM_BYPASS_MODE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Select default output data rate. */
  pObj->acc_odr = LSM6DSM_XL_ODR_104Hz;

  /* Output data rate selection - power down. */
  if (lsm6dsm_xl_data_rate_set(&(pObj->Ctx), LSM6DSM_XL_ODR_OFF) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dsm_xl_full_scale_set(&(pObj->Ctx), LSM6DSM_2g) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Select default output data rate. */
  pObj->gyro_odr = LSM6DSM_GY_ODR_104Hz;

  /* Output data rate selection - power down. */
  if (lsm6dsm_gy_data_rate_set(&(pObj->Ctx), LSM6DSM_GY_ODR_OFF) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dsm_gy_full_scale_set(&(pObj->Ctx), LSM6DSM_2000dps) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  pObj->is_initialized = 1;

  return LSM6DSM_OK;
}

/**
 * @brief  Deinitialize the LSM6DSM sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_DeInit(LSM6DSM_Object_t *pObj)
{
  /* Disable the component */
  if (LSM6DSM_ACC_Disable(pObj) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (LSM6DSM_GYRO_Disable(pObj) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Reset output data rate. */
  pObj->acc_odr = LSM6DSM_XL_ODR_OFF;
  pObj->gyro_odr = LSM6DSM_GY_ODR_OFF;

  pObj->is_initialized = 0;

  return LSM6DSM_OK;
}

/**
 * @brief  Read component ID
 * @param  pObj the device pObj
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ReadID(LSM6DSM_Object_t *pObj, uint8_t *Id)
{
  if (lsm6dsm_device_id_get(&(pObj->Ctx), Id) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Get LSM6DSM sensor capabilities
 * @param  pObj Component object pointer
 * @param  Capabilities pointer to LSM6DSM sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GetCapabilities(LSM6DSM_Object_t *pObj, LSM6DSM_Capabilities_t *Capabilities)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(pObj);

  Capabilities->Acc          = 1;
  Capabilities->Gyro         = 1;
  Capabilities->Magneto      = 0;
  Capabilities->LowPower     = 0;
  Capabilities->GyroMaxFS    = 2000;
  Capabilities->AccMaxFS     = 16;
  Capabilities->MagMaxFS     = 0;
  Capabilities->GyroMaxOdr   = 6660.0f;
  Capabilities->AccMaxOdr    = 6660.0f;
  Capabilities->MagMaxOdr    = 0.0f;
  return LSM6DSM_OK;
}

/**
 * @brief  Enable the LSM6DSM accelerometer sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Enable(LSM6DSM_Object_t *pObj)
{
  /* Check if the component is already enabled */
  if (pObj->acc_is_enabled == 1U)
  {
    return LSM6DSM_OK;
  }

  /* Output data rate selection. */
  if (lsm6dsm_xl_data_rate_set(&(pObj->Ctx), pObj->acc_odr) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  pObj->acc_is_enabled = 1;

  return LSM6DSM_OK;
}

/**
 * @brief  Disable the LSM6DSM accelerometer sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Disable(LSM6DSM_Object_t *pObj)
{
  /* Check if the component is already disabled */
  if (pObj->acc_is_enabled == 0U)
  {
    return LSM6DSM_OK;
  }

  /* Get current output data rate. */
  if (lsm6dsm_xl_data_rate_get(&(pObj->Ctx), &pObj->acc_odr) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm6dsm_xl_data_rate_set(&(pObj->Ctx), LSM6DSM_XL_ODR_OFF) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  pObj->acc_is_enabled = 0;

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM accelerometer sensor sensitivity
 * @param  pObj the device pObj
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_GetSensitivity(LSM6DSM_Object_t *pObj, float *Sensitivity)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_fs_xl_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsm_xl_full_scale_get(&(pObj->Ctx), &full_scale) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Store the Sensitivity based on actual full scale. */
  switch (full_scale)
  {
    case LSM6DSM_2g:
      *Sensitivity = LSM6DSM_ACC_SENSITIVITY_FS_2G;
      break;

    case LSM6DSM_4g:
      *Sensitivity = LSM6DSM_ACC_SENSITIVITY_FS_4G;
      break;

    case LSM6DSM_8g:
      *Sensitivity = LSM6DSM_ACC_SENSITIVITY_FS_8G;
      break;

    case LSM6DSM_16g:
      *Sensitivity = LSM6DSM_ACC_SENSITIVITY_FS_16G;
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSM accelerometer sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_GetOutputDataRate(LSM6DSM_Object_t *pObj, float *Odr)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_odr_xl_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dsm_xl_data_rate_get(&(pObj->Ctx), &odr_low_level) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  switch (odr_low_level)
  {
    case LSM6DSM_XL_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSM_XL_ODR_1Hz6:
      *Odr = 1.6f;
      break;

    case LSM6DSM_XL_ODR_12Hz5:
      *Odr = 12.5f;
      break;

    case LSM6DSM_XL_ODR_26Hz:
      *Odr = 26.0f;
      break;

    case LSM6DSM_XL_ODR_52Hz:
      *Odr = 52.0f;
      break;

    case LSM6DSM_XL_ODR_104Hz:
      *Odr = 104.0f;
      break;

    case LSM6DSM_XL_ODR_208Hz:
      *Odr = 208.0f;
      break;

    case LSM6DSM_XL_ODR_416Hz:
      *Odr = 416.0f;
      break;

    case LSM6DSM_XL_ODR_833Hz:
      *Odr = 833.0f;
      break;

    case LSM6DSM_XL_ODR_1k66Hz:
      *Odr = 1660.0f;
      break;

    case LSM6DSM_XL_ODR_3k33Hz:
      *Odr = 3330.0f;
      break;

    case LSM6DSM_XL_ODR_6k66Hz:
      *Odr = 6660.0f;
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSM accelerometer sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_SetOutputDataRate(LSM6DSM_Object_t *pObj, float Odr)
{
  /* Check if the component is enabled */
  if (pObj->acc_is_enabled == 1U)
  {
    return LSM6DSM_ACC_SetOutputDataRate_When_Enabled(pObj, Odr);
  }
  else
  {
    return LSM6DSM_ACC_SetOutputDataRate_When_Disabled(pObj, Odr);
  }
}

/**
 * @brief  Get the LSM6DSM accelerometer sensor full scale
 * @param  pObj the device pObj
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_GetFullScale(LSM6DSM_Object_t *pObj, int32_t *FullScale)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_fs_xl_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsm_xl_full_scale_get(&(pObj->Ctx), &fs_low_level) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  switch (fs_low_level)
  {
    case LSM6DSM_2g:
      *FullScale =  2;
      break;

    case LSM6DSM_4g:
      *FullScale =  4;
      break;

    case LSM6DSM_8g:
      *FullScale =  8;
      break;

    case LSM6DSM_16g:
      *FullScale = 16;
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSM accelerometer sensor full scale
 * @param  pObj the device pObj
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_SetFullScale(LSM6DSM_Object_t *pObj, int32_t FullScale)
{
  lsm6dsm_fs_xl_t new_fs;

  /* Seems like MISRA C-2012 rule 14.3a violation but only from single file statical analysis point of view because
     the parameter passed to the function is not known at the moment of analysis */
  new_fs = (FullScale <= 2) ? LSM6DSM_2g
           : (FullScale <= 4) ? LSM6DSM_4g
           : (FullScale <= 8) ? LSM6DSM_8g
           :                    LSM6DSM_16g;

  if (lsm6dsm_xl_full_scale_set(&(pObj->Ctx), new_fs) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM accelerometer sensor raw axes
 * @param  pObj the device pObj
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_GetAxesRaw(LSM6DSM_Object_t *pObj, LSM6DSM_AxesRaw_t *Value)
{
  lsm6dsm_axis3bit16_t  data_raw;

  /* Read raw data values. */
  if (lsm6dsm_acceleration_raw_get(&(pObj->Ctx), data_raw.u8bit) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Format the data. */
  Value->x = data_raw.i16bit[0];
  Value->y = data_raw.i16bit[1];
  Value->z = data_raw.i16bit[2];

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM accelerometer sensor axes
 * @param  pObj the device pObj
 * @param  Acceleration pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_GetAxes(LSM6DSM_Object_t *pObj, LSM6DSM_Axes_t *Acceleration)
{
  lsm6dsm_axis3bit16_t  data_raw;
  float sensitivity = 0.0f;

  /* Read raw data values. */
  if (lsm6dsm_acceleration_raw_get(&(pObj->Ctx), data_raw.u8bit) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Get LSM6DSM actual sensitivity. */
  if (LSM6DSM_ACC_GetSensitivity(pObj, &sensitivity) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Calculate the data. */
  Acceleration->x = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  Acceleration->y = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  Acceleration->z = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LSM6DSM_OK;
}

/**
 * @brief  Enable the LSM6DSM gyroscope sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GYRO_Enable(LSM6DSM_Object_t *pObj)
{
  /* Check if the component is already enabled */
  if (pObj->gyro_is_enabled == 1U)
  {
    return LSM6DSM_OK;
  }

  /* Output data rate selection. */
  if (lsm6dsm_gy_data_rate_set(&(pObj->Ctx), pObj->gyro_odr) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  pObj->gyro_is_enabled = 1;

  return LSM6DSM_OK;
}

/**
 * @brief  Disable the LSM6DSM gyroscope sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GYRO_Disable(LSM6DSM_Object_t *pObj)
{
  /* Check if the component is already disabled */
  if (pObj->gyro_is_enabled == 0U)
  {
    return LSM6DSM_OK;
  }

  /* Get current output data rate. */
  if (lsm6dsm_gy_data_rate_get(&(pObj->Ctx), &pObj->gyro_odr) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lsm6dsm_gy_data_rate_set(&(pObj->Ctx), LSM6DSM_GY_ODR_OFF) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  pObj->gyro_is_enabled = 0;

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM gyroscope sensor sensitivity
 * @param  pObj the device pObj
 * @param  Sensitivity pointer
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GYRO_GetSensitivity(LSM6DSM_Object_t *pObj, float *Sensitivity)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_fs_g_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsm_gy_full_scale_get(&(pObj->Ctx), &full_scale) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (full_scale)
  {
    case LSM6DSM_125dps:
      *Sensitivity = LSM6DSM_GYRO_SENSITIVITY_FS_125DPS;
      break;

    case LSM6DSM_250dps:
      *Sensitivity = LSM6DSM_GYRO_SENSITIVITY_FS_250DPS;
      break;

    case LSM6DSM_500dps:
      *Sensitivity = LSM6DSM_GYRO_SENSITIVITY_FS_500DPS;
      break;

    case LSM6DSM_1000dps:
      *Sensitivity = LSM6DSM_GYRO_SENSITIVITY_FS_1000DPS;
      break;

    case LSM6DSM_2000dps:
      *Sensitivity = LSM6DSM_GYRO_SENSITIVITY_FS_2000DPS;
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSM gyroscope sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GYRO_GetOutputDataRate(LSM6DSM_Object_t *pObj, float *Odr)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_odr_g_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dsm_gy_data_rate_get(&(pObj->Ctx), &odr_low_level) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  switch (odr_low_level)
  {
    case LSM6DSM_GY_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSM_GY_ODR_12Hz5:
      *Odr = 12.5f;
      break;

    case LSM6DSM_GY_ODR_26Hz:
      *Odr = 26.0f;
      break;

    case LSM6DSM_GY_ODR_52Hz:
      *Odr = 52.0f;
      break;

    case LSM6DSM_GY_ODR_104Hz:
      *Odr = 104.0f;
      break;

    case LSM6DSM_GY_ODR_208Hz:
      *Odr = 208.0f;
      break;

    case LSM6DSM_GY_ODR_416Hz:
      *Odr = 416.0f;
      break;

    case LSM6DSM_GY_ODR_833Hz:
      *Odr = 833.0f;
      break;

    case LSM6DSM_GY_ODR_1k66Hz:
      *Odr =  1660.0f;
      break;

    case LSM6DSM_GY_ODR_3k33Hz:
      *Odr =  3330.0f;
      break;

    case LSM6DSM_GY_ODR_6k66Hz:
      *Odr =  6660.0f;
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSM gyroscope sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GYRO_SetOutputDataRate(LSM6DSM_Object_t *pObj, float Odr)
{
  /* Check if the component is enabled */
  if (pObj->gyro_is_enabled == 1U)
  {
    return LSM6DSM_GYRO_SetOutputDataRate_When_Enabled(pObj, Odr);
  }
  else
  {
    return LSM6DSM_GYRO_SetOutputDataRate_When_Disabled(pObj, Odr);
  }
}

/**
 * @brief  Get the LSM6DSM gyroscope sensor full scale
 * @param  pObj the device pObj
 * @param  FullScale pointer where the full scale is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GYRO_GetFullScale(LSM6DSM_Object_t *pObj, int32_t  *FullScale)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_fs_g_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dsm_gy_full_scale_get(&(pObj->Ctx), &fs_low_level) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  switch (fs_low_level)
  {
    case LSM6DSM_125dps:
      *FullScale =  125;
      break;

    case LSM6DSM_250dps:
      *FullScale =  250;
      break;

    case LSM6DSM_500dps:
      *FullScale =  500;
      break;

    case LSM6DSM_1000dps:
      *FullScale = 1000;
      break;

    case LSM6DSM_2000dps:
      *FullScale = 2000;
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the LSM6DSM gyroscope sensor full scale
 * @param  pObj the device pObj
 * @param  FullScale the functional full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GYRO_SetFullScale(LSM6DSM_Object_t *pObj, int32_t FullScale)
{
  lsm6dsm_fs_g_t new_fs;

  new_fs = (FullScale <= 125)  ? LSM6DSM_125dps
           : (FullScale <= 250)  ? LSM6DSM_250dps
           : (FullScale <= 500)  ? LSM6DSM_500dps
           : (FullScale <= 1000) ? LSM6DSM_1000dps
           :                       LSM6DSM_2000dps;

  if (lsm6dsm_gy_full_scale_set(&(pObj->Ctx), new_fs) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM gyroscope sensor raw axes
 * @param  pObj the device pObj
 * @param  Value pointer where the raw values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GYRO_GetAxesRaw(LSM6DSM_Object_t *pObj, LSM6DSM_AxesRaw_t *Value)
{
  lsm6dsm_axis3bit16_t  data_raw;

  /* Read raw data values. */
  if (lsm6dsm_angular_rate_raw_get(&(pObj->Ctx), data_raw.u8bit) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Format the data. */
  Value->x = data_raw.i16bit[0];
  Value->y = data_raw.i16bit[1];
  Value->z = data_raw.i16bit[2];

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM gyroscope sensor axes
 * @param  pObj the device pObj
 * @param  AngularRate pointer where the values of the axes are written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GYRO_GetAxes(LSM6DSM_Object_t *pObj, LSM6DSM_Axes_t *AngularRate)
{
  lsm6dsm_axis3bit16_t  data_raw;
  float sensitivity;

  /* Read raw data values. */
  if (lsm6dsm_angular_rate_raw_get(&(pObj->Ctx), data_raw.u8bit) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Get LSM6DSM actual sensitivity. */
  if (LSM6DSM_GYRO_GetSensitivity(pObj, &sensitivity) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Calculate the data. */
  AngularRate->x = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  AngularRate->y = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  AngularRate->z = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM register value
 * @param  pObj the device pObj
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_Read_Reg(LSM6DSM_Object_t *pObj, uint8_t Reg, uint8_t *Data)
{
  if (lsm6dsm_read_reg(&(pObj->Ctx), Reg, Data, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set the LSM6DSM register value
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_Write_Reg(LSM6DSM_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  if (lsm6dsm_write_reg(&(pObj->Ctx), Reg, &Data, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set the interrupt latch
 * @param  pObj the device pObj
 * @param  Status value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_Set_Interrupt_Latch(LSM6DSM_Object_t *pObj, uint8_t Status)
{
  if (Status > 1U)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_int_notification_set(&(pObj->Ctx), (lsm6dsm_lir_t)Status) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Enable free fall detection
 * @param  pObj the device pObj
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Enable_Free_Fall_Detection(LSM6DSM_Object_t *pObj, LSM6DSM_SensorIntPin_t IntPin)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Output Data Rate selection */
  if (LSM6DSM_ACC_SetOutputDataRate(pObj, 416.0f) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Full scale selection */
  if (LSM6DSM_ACC_SetFullScale(pObj, 2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* FF_DUR setting */
  if (lsm6dsm_ff_dur_set(&(pObj->Ctx), 0x06) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* WAKE_DUR setting */
  if (lsm6dsm_wkup_dur_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* TIMER_HR setting */
  if (lsm6dsm_timestamp_res_set(&(pObj->Ctx), LSM6DSM_LSB_6ms4) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* SLEEP_DUR setting */
  if (lsm6dsm_act_sleep_dur_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* FF_THS setting */
  if (lsm6dsm_ff_threshold_set(&(pObj->Ctx), LSM6DSM_FF_TSH_312mg) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable free fall event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSM_INT1_PIN:
      if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val1.int1_ff = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    case LSM6DSM_INT2_PIN:
      if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val2.int2_ff = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable free fall detection
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Disable_Free_Fall_Detection(LSM6DSM_Object_t *pObj)
{
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Disable free fall event on both INT1 and INT2 pins */
  if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val1.int1_ff = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val2.int2_ff = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* FF_DUR setting */
  if (lsm6dsm_ff_dur_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* FF_THS setting */
  if (lsm6dsm_ff_threshold_set(&(pObj->Ctx), LSM6DSM_FF_TSH_156mg) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set free fall threshold
 * @param  pObj the device pObj
 * @param  Threshold free fall detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Set_Free_Fall_Threshold(LSM6DSM_Object_t *pObj, uint8_t Threshold)
{
  if (lsm6dsm_ff_threshold_set(&(pObj->Ctx), (lsm6dsm_ff_ths_t)Threshold) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set free fall duration
 * @param  pObj the device pObj
 * @param  Duration free fall detection duration
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Set_Free_Fall_Duration(LSM6DSM_Object_t *pObj, uint8_t Duration)
{
  if (lsm6dsm_ff_dur_set(&(pObj->Ctx), Duration) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
* @brief  Enable pedometer
* @param  pObj the device pObj
* @retval 0 in case of success, an error code otherwise
*/
int32_t LSM6DSM_ACC_Enable_Pedometer(LSM6DSM_Object_t *pObj, LSM6DSM_SensorIntPin_t IntPin)
{
    lsm6dsm_int1_route_t val_1;
    lsm6dsm_int2_route_t val_2;
  
  /* Output Data Rate selection */
  if (LSM6DSM_ACC_SetOutputDataRate(pObj, 26.0f) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }
  
  /* Full scale selection */
  if (LSM6DSM_ACC_SetFullScale(pObj, 2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }
  
  /* Set pedometer threshold. */
  if (lsm6dsm_pedo_threshold_set(&(pObj->Ctx), 0x17) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }
  
  /* Enable pedometer algorithm. */
  if (lsm6dsm_pedo_sens_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }
  
  switch (IntPin)
  {
  case LSM6DSM_INT1_PIN:
    
    /* Enable pedometer on INT1 pin */
    if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val_1) != LSM6DSM_OK)
    {
      return LSM6DSM_ERROR;
    }
    
    val_1.int1_step_detector = PROPERTY_ENABLE;
    
    if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val_1) != LSM6DSM_OK)
    {
      return LSM6DSM_ERROR;
    }
    
    break;
    
  case LSM6DSM_INT2_PIN:
    
    /* Enable pedometer on INT2 pin */
    if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val_2) != LSM6DSM_OK)
    {
      return LSM6DSM_ERROR;
    }
    
    val_2.int2_step_delta = PROPERTY_ENABLE;
    
    if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val_2) != LSM6DSM_OK)
    {
      return LSM6DSM_ERROR;
    }
    break;
  }
  
  return LSM6DSM_OK;
}

/**
 * @brief  Disable pedometer
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Disable_Pedometer(LSM6DSM_Object_t *pObj)
{
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val_2;
  
  /* Disable step detector on INT1 pin */
  if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }
  
  val1.int1_step_detector = PROPERTY_DISABLE;
  
  if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }
  
  /* Disable pedometer on INT2 pin */
  if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val_2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }
  
  val_2.int2_step_delta = PROPERTY_DISABLE;
  
  if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val_2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }
  
  /* Disable pedometer algorithm. */
  if (lsm6dsm_pedo_sens_set(&(pObj->Ctx), PROPERTY_DISABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Reset pedometer threshold. */
  if (lsm6dsm_pedo_threshold_set(&(pObj->Ctx), 0x0) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Get step count
 * @param  pObj the device pObj
 * @param  StepCount step counter
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Get_Step_Count(LSM6DSM_Object_t *pObj, uint16_t *StepCount)
{
  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_STEP_COUNTER_L, (uint8_t *)StepCount, 2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Enable step counter reset
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Enable_Step_Counter_Reset(LSM6DSM_Object_t *pObj)
{
  if (lsm6dsm_pedo_step_reset_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Disable step counter reset
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Disable_Step_Counter_Reset(LSM6DSM_Object_t *pObj)
{
  if (lsm6dsm_pedo_step_reset_set(&(pObj->Ctx), PROPERTY_DISABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set pedometer threshold
 * @param  pObj the device pObj
 * @param  Threshold pedometer threshold
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Set_Pedometer_Threshold(LSM6DSM_Object_t *pObj, uint8_t Threshold)
{
  if (lsm6dsm_pedo_threshold_set(&(pObj->Ctx), Threshold) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Enable tilt detection
 * @param  pObj the device pObj
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Enable_Tilt_Detection(LSM6DSM_Object_t *pObj, LSM6DSM_SensorIntPin_t IntPin)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Output Data Rate selection */
  if (LSM6DSM_ACC_SetOutputDataRate(pObj, 26.0f) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Full scale selection */
  if (LSM6DSM_ACC_SetFullScale(pObj, 2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable tilt calculation. */
  if (lsm6dsm_tilt_sens_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable tilt event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSM_INT1_PIN:
      if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val1.int1_tilt = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    case LSM6DSM_INT2_PIN:
      if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val2.int2_tilt = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable tilt detection
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Disable_Tilt_Detection(LSM6DSM_Object_t *pObj)
{
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Disable tilt event on both INT1 and INT2 pins */
  if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val1.int1_tilt = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val2.int2_tilt = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Disable tilt calculation. */
  if (lsm6dsm_tilt_sens_set(&(pObj->Ctx), PROPERTY_DISABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Enable wake up detection
 * @param  pObj the device pObj
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Enable_Wake_Up_Detection(LSM6DSM_Object_t *pObj, LSM6DSM_SensorIntPin_t IntPin)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Output Data Rate selection */
  if (LSM6DSM_ACC_SetOutputDataRate(pObj, 416.0f) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Full scale selection */
  if (LSM6DSM_ACC_SetFullScale(pObj, 2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* WAKE_DUR setting */
  if (lsm6dsm_wkup_dur_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Set wake up threshold. */
  if (lsm6dsm_wkup_threshold_set(&(pObj->Ctx), 0x02) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable wake up event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSM_INT1_PIN:
      if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val1.int1_wu = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    case LSM6DSM_INT2_PIN:
      if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val2.int2_wu = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable wake up detection
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Disable_Wake_Up_Detection(LSM6DSM_Object_t *pObj)
{
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Disable wake up event on both INT1 and INT2 pins */
  if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val1.int1_wu = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val2.int2_wu = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Reset wake up threshold. */
  if (lsm6dsm_wkup_threshold_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* WAKE_DUR setting */
  if (lsm6dsm_wkup_dur_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set wake up threshold
 * @param  pObj the device pObj
 * @param  Threshold wake up detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Set_Wake_Up_Threshold(LSM6DSM_Object_t *pObj, uint8_t Threshold)
{
  /* Set wake up threshold. */
  if (lsm6dsm_wkup_threshold_set(&(pObj->Ctx), Threshold) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set wake up duration
 * @param  pObj the device pObj
 * @param  Duration wake up detection duration
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Set_Wake_Up_Duration(LSM6DSM_Object_t *pObj, uint8_t Duration)
{
  /* Set wake up duration. */
  if (lsm6dsm_wkup_dur_set(&(pObj->Ctx), Duration) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Enable inactivity detection
 * @param  pObj the device pObj
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Enable_Inactivity_Detection(LSM6DSM_Object_t *pObj, LSM6DSM_SensorIntPin_t IntPin)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Output Data Rate and Full scale must be selected externally */

  /* SLEEP_DUR setting */
  if (lsm6dsm_act_sleep_dur_set(&(pObj->Ctx), 0x01) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Set wake up threshold. */
  if (lsm6dsm_wkup_threshold_set(&(pObj->Ctx), 0x02) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable inactivity detection. Gyroscope is left configured as it is */
  if (lsm6dsm_act_mode_set(&(pObj->Ctx), LSM6DSM_XL_12Hz5_GY_NOT_AFFECTED) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable activity/inactivity event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSM_INT1_PIN:
      if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val1.int1_inact_state = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    case LSM6DSM_INT2_PIN:
      if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val2.int2_inact_state = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable inactivity detection
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Disable_Inactivity_Detection(LSM6DSM_Object_t *pObj)
{
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Disable activity/inactivity event on both INT1 and INT2 pins */
  if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val1.int1_inact_state = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val2.int2_inact_state = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Disable inactivity detection. */
  if (lsm6dsm_act_mode_set(&(pObj->Ctx), LSM6DSM_PROPERTY_DISABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Reset wake up threshold. */
  if (lsm6dsm_wkup_threshold_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* SLEEP_DUR setting */
  if (lsm6dsm_act_sleep_dur_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set sleep duration
 * @param  pObj the device pObj
 * @param  Duration wake up detection duration
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Set_Sleep_Duration(LSM6DSM_Object_t *pObj, uint8_t Duration)
{
  /* Set sleep duration. */
  if (lsm6dsm_act_sleep_dur_set(&(pObj->Ctx), Duration) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Enable single tap detection
 * @param  pObj the device pObj
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Enable_Single_Tap_Detection(LSM6DSM_Object_t *pObj, LSM6DSM_SensorIntPin_t IntPin)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Output Data Rate selection */
  if (LSM6DSM_ACC_SetOutputDataRate(pObj, 416.0f) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Full scale selection */
  if (LSM6DSM_ACC_SetFullScale(pObj, 2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if (lsm6dsm_tap_detection_on_x_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if (lsm6dsm_tap_detection_on_y_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if (lsm6dsm_tap_detection_on_z_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Set tap threshold. */
  if (lsm6dsm_tap_threshold_x_set(&(pObj->Ctx), 0x08) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Set tap shock time window. */
  if (lsm6dsm_tap_shock_set(&(pObj->Ctx), 0x02) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Set tap quiet time window. */
  if (lsm6dsm_tap_quiet_set(&(pObj->Ctx), 0x01) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* _NOTE_: Tap duration time window - don't care for single tap. */

  /* _NOTE_: Single/Double Tap event - don't care of this flag for single tap. */

  /* Enable single tap event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSM_INT1_PIN:
      if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val1.int1_single_tap = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    case LSM6DSM_INT2_PIN:
      if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val2.int2_single_tap = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable single tap detection
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Disable_Single_Tap_Detection(LSM6DSM_Object_t *pObj)
{
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Disable single tap event on both INT1 and INT2 pins */
  if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val1.int1_single_tap = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val2.int2_single_tap = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Reset tap quiet time window. */
  if (lsm6dsm_tap_quiet_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Reset tap shock time window. */
  if (lsm6dsm_tap_shock_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Reset tap threshold. */
  if (lsm6dsm_tap_threshold_x_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Disable Z direction in tap recognition. */
  if (lsm6dsm_tap_detection_on_z_set(&(pObj->Ctx), PROPERTY_DISABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if (lsm6dsm_tap_detection_on_y_set(&(pObj->Ctx), PROPERTY_DISABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if (lsm6dsm_tap_detection_on_x_set(&(pObj->Ctx), PROPERTY_DISABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Enable double tap detection
 * @param  pObj the device pObj
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Enable_Double_Tap_Detection(LSM6DSM_Object_t *pObj, LSM6DSM_SensorIntPin_t IntPin)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Output Data Rate selection */
  if (LSM6DSM_ACC_SetOutputDataRate(pObj, 416.0f) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Full scale selection */
  if (LSM6DSM_ACC_SetFullScale(pObj, 2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if (lsm6dsm_tap_detection_on_x_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if (lsm6dsm_tap_detection_on_y_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if (lsm6dsm_tap_detection_on_z_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Set tap threshold. */
  if (lsm6dsm_tap_threshold_x_set(&(pObj->Ctx), 0x08) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Set tap shock time window. */
  if (lsm6dsm_tap_shock_set(&(pObj->Ctx), 0x03) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Set tap quiet time window. */
  if (lsm6dsm_tap_quiet_set(&(pObj->Ctx), 0x03) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Set tap duration time window. */
  if (lsm6dsm_tap_dur_set(&(pObj->Ctx), 0x08) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Single and double tap enabled. */
  if (lsm6dsm_tap_mode_set(&(pObj->Ctx), LSM6DSM_BOTH_SINGLE_DOUBLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable double tap event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSM_INT1_PIN:
      if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val1.int1_double_tap = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    case LSM6DSM_INT2_PIN:
      if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val2.int2_double_tap = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable double tap detection
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Disable_Double_Tap_Detection(LSM6DSM_Object_t *pObj)
{
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Disable double tap event on both INT1 and INT2 pins */
  if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val1.int1_double_tap = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val2.int2_double_tap = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Only single tap enabled. */
  if (lsm6dsm_tap_mode_set(&(pObj->Ctx), LSM6DSM_ONLY_SINGLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Reset tap duration time window. */
  if (lsm6dsm_tap_dur_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Reset tap quiet time window. */
  if (lsm6dsm_tap_quiet_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Reset tap shock time window. */
  if (lsm6dsm_tap_shock_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Reset tap threshold. */
  if (lsm6dsm_tap_threshold_x_set(&(pObj->Ctx), 0x00) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Disable Z direction in tap recognition. */
  if (lsm6dsm_tap_detection_on_z_set(&(pObj->Ctx), PROPERTY_DISABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if (lsm6dsm_tap_detection_on_y_set(&(pObj->Ctx), PROPERTY_DISABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if (lsm6dsm_tap_detection_on_x_set(&(pObj->Ctx), PROPERTY_DISABLE) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set tap threshold
 * @param  pObj the device pObj
 * @param  Threshold tap threshold
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Set_Tap_Threshold(LSM6DSM_Object_t *pObj, uint8_t Threshold)
{
  /* Set tap threshold. */
  if (lsm6dsm_tap_threshold_x_set(&(pObj->Ctx), Threshold) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set tap shock time
 * @param  pObj the device pObj
 * @param  Time tap shock time
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Set_Tap_Shock_Time(LSM6DSM_Object_t *pObj, uint8_t Time)
{
  /* Set tap shock time window. */
  if (lsm6dsm_tap_shock_set(&(pObj->Ctx), Time) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set tap quiet time
 * @param  pObj the device pObj
 * @param  Time tap quiet time
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Set_Tap_Quiet_Time(LSM6DSM_Object_t *pObj, uint8_t Time)
{
  /* Set tap quiet time window. */
  if (lsm6dsm_tap_quiet_set(&(pObj->Ctx), Time) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set tap duration time
 * @param  pObj the device pObj
 * @param  Time tap duration time
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Set_Tap_Duration_Time(LSM6DSM_Object_t *pObj, uint8_t Time)
{
  /* Set tap duration time window. */
  if (lsm6dsm_tap_dur_set(&(pObj->Ctx), Time) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Enable 6D orientation detection
 * @param  pObj the device pObj
 * @param  IntPin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Enable_6D_Orientation(LSM6DSM_Object_t *pObj, LSM6DSM_SensorIntPin_t IntPin)
{
  int32_t ret = LSM6DSM_OK;
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Output Data Rate selection */
  if (LSM6DSM_ACC_SetOutputDataRate(pObj, 416.0f) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Full scale selection */
  if (LSM6DSM_ACC_SetFullScale(pObj, 2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* 6D orientation enabled. */
  if (lsm6dsm_6d_threshold_set(&(pObj->Ctx), LSM6DSM_DEG_60) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Enable 6D orientation event on either INT1 or INT2 pin */
  switch (IntPin)
  {
    case LSM6DSM_INT1_PIN:
      if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val1.int1_6d = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    case LSM6DSM_INT2_PIN:
      if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }

      val2.int2_6d = PROPERTY_ENABLE;

      if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
      {
        return LSM6DSM_ERROR;
      }
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable 6D orientation detection
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Disable_6D_Orientation(LSM6DSM_Object_t *pObj)
{
  lsm6dsm_int1_route_t val1;
  lsm6dsm_int2_route_t val2;

  /* Disable 6D orientation event on both INT1 and INT2 pins */
  if (lsm6dsm_pin_int1_route_get(&(pObj->Ctx), &val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val1.int1_6d = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int1_route_set(&(pObj->Ctx), val1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_pin_int2_route_get(&(pObj->Ctx), &val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  val2.int2_6d = PROPERTY_DISABLE;

  if (lsm6dsm_pin_int2_route_set(&(pObj->Ctx), val2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  /* Reset 6D orientation. */
  if (lsm6dsm_6d_threshold_set(&(pObj->Ctx), LSM6DSM_DEG_80) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set 6D orientation threshold
 * @param  pObj the device pObj
 * @param  Threshold free fall detection threshold
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Set_6D_Orientation_Threshold(LSM6DSM_Object_t *pObj, uint8_t Threshold)
{
  if (lsm6dsm_6d_threshold_set(&(pObj->Ctx), (lsm6dsm_sixd_ths_t)Threshold) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Get the status of XLow orientation
 * @param  pObj the device pObj
 * @param  XLow the status of XLow orientation
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Get_6D_Orientation_XL(LSM6DSM_Object_t *pObj, uint8_t *XLow)
{
  lsm6dsm_d6d_src_t data;

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  *XLow = data.xl;

  return LSM6DSM_OK;
}

/**
 * @brief  Get the status of XHigh orientation
 * @param  pObj the device pObj
 * @param  XHigh the status of XHigh orientation
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Get_6D_Orientation_XH(LSM6DSM_Object_t *pObj, uint8_t *XHigh)
{
  lsm6dsm_d6d_src_t data;

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  *XHigh = data.xh;

  return LSM6DSM_OK;
}

/**
 * @brief  Get the status of YLow orientation
 * @param  pObj the device pObj
 * @param  YLow the status of YLow orientation
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Get_6D_Orientation_YL(LSM6DSM_Object_t *pObj, uint8_t *YLow)
{
  lsm6dsm_d6d_src_t data;

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  *YLow = data.yl;

  return LSM6DSM_OK;
}

/**
 * @brief  Get the status of YHigh orientation
 * @param  pObj the device pObj
 * @param  YHigh the status of YHigh orientation
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Get_6D_Orientation_YH(LSM6DSM_Object_t *pObj, uint8_t *YHigh)
{
  lsm6dsm_d6d_src_t data;

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  *YHigh = data.yh;

  return LSM6DSM_OK;
}

/**
 * @brief  Get the status of ZLow orientation
 * @param  pObj the device pObj
 * @param  ZLow the status of ZLow orientation
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Get_6D_Orientation_ZL(LSM6DSM_Object_t *pObj, uint8_t *ZLow)
{
  lsm6dsm_d6d_src_t data;

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  *ZLow = data.zl;

  return LSM6DSM_OK;
}

/**
 * @brief  Get the status of ZHigh orientation
 * @param  pObj the device pObj
 * @param  ZHigh the status of ZHigh orientation
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Get_6D_Orientation_ZH(LSM6DSM_Object_t *pObj, uint8_t *ZHigh)
{
  lsm6dsm_d6d_src_t data;

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_D6D_SRC, (uint8_t *)&data, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  *ZHigh = data.zh;

  return LSM6DSM_OK;
}

/**
 * @brief  Get the status of all hardware events
 * @param  pObj the device pObj
 * @param  Status the status of all hardware events
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Get_Event_Status(LSM6DSM_Object_t *pObj, LSM6DSM_Event_Status_t *Status)
{
  lsm6dsm_wake_up_src_t wake_up_src;
  lsm6dsm_tap_src_t tap_src;
  lsm6dsm_d6d_src_t d6d_src;
  lsm6dsm_func_src1_t func_src;
  lsm6dsm_md1_cfg_t md1_cfg;
  lsm6dsm_md2_cfg_t md2_cfg;
  lsm6dsm_int1_ctrl_t int1_ctrl;
  lsm6dsm_int2_ctrl_t int2_ctrl;

  (void)memset((void *)Status, 0x0, sizeof(LSM6DSM_Event_Status_t));

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_WAKE_UP_SRC, (uint8_t *)&wake_up_src, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_TAP_SRC, (uint8_t *)&tap_src, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_D6D_SRC, (uint8_t *)&d6d_src, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_FUNC_SRC1, (uint8_t *)&func_src, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_MD1_CFG, (uint8_t *)&md1_cfg, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_MD2_CFG, (uint8_t *)&md2_cfg, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_INT1_CTRL, (uint8_t *)&int1_ctrl, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }
  
  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_INT2_CTRL, (uint8_t *)&int2_ctrl, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  if ((md1_cfg.int1_ff == 1U) || (md2_cfg.int2_ff == 1U))
  {
    if (wake_up_src.ff_ia == 1U)
    {
      Status->FreeFallStatus = 1;
    }
  }

  if ((md1_cfg.int1_wu == 1U) || (md2_cfg.int2_wu == 1U))
  {
    if (wake_up_src.wu_ia == 1U)
    {
      Status->WakeUpStatus = 1;
    }
  }

  if ((md1_cfg.int1_inact_state == 1U) || (md2_cfg.int2_inact_state == 1U))
  {
    if (wake_up_src.sleep_state_ia == 1U)
    {
      Status->SleepStatus = 1;
    }
  }

  if ((md1_cfg.int1_single_tap == 1U) || (md2_cfg.int2_single_tap == 1U))
  {
    if (tap_src.single_tap == 1U)
    {
      Status->TapStatus = 1;
    }
  }

  if ((md1_cfg.int1_double_tap == 1U) || (md2_cfg.int2_double_tap == 1U))
  {
    if (tap_src.double_tap == 1U)
    {
      Status->DoubleTapStatus = 1;
    }
  }

  if ((md1_cfg.int1_6d == 1U) || (md2_cfg.int2_6d == 1U))
  {
    if (d6d_src.d6d_ia == 1U)
    {
      Status->D6DOrientationStatus = 1;
    }
  }

  if ((int1_ctrl.int1_step_detector == 1U) || (int2_ctrl.int2_step_delta == 1U))
  {
    if (func_src.step_detected == 1U)
    {
      Status->StepStatus = 1;
    }
  }

  if ((md1_cfg.int1_tilt == 1U) || (md2_cfg.int2_tilt == 1U))
  {
    if (func_src.tilt_ia == 1U)
    {
      Status->TiltStatus = 1;
    }
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set self test
 * @param  pObj the device pObj
 * @param  val the value of st_xl in reg CTRL5_C
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Set_SelfTest(LSM6DSM_Object_t *pObj, uint8_t val)
{
  lsm6dsm_st_xl_t reg;

  reg = (val == 0U)  ? LSM6DSM_XL_ST_DISABLE
        : (val == 1U)  ? LSM6DSM_XL_ST_POSITIVE
        : (val == 2U)  ? LSM6DSM_XL_ST_NEGATIVE
        :                LSM6DSM_XL_ST_DISABLE;

  if (lsm6dsm_xl_self_test_set(&(pObj->Ctx), reg) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM ACC data ready bit value
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Get_DRDY_Status(LSM6DSM_Object_t *pObj, uint8_t *Status)
{
  if (lsm6dsm_xl_flag_data_ready_get(&(pObj->Ctx), Status) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM ACC initialization status
 * @param  pObj the device pObj
 * @param  Status 1 if initialized, 0 otherwise
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_ACC_Get_Init_Status(LSM6DSM_Object_t *pObj, uint8_t *Status)
{
  if (pObj == NULL)
  {
    return LSM6DSM_ERROR;
  }

  *Status = pObj->is_initialized;

  return LSM6DSM_OK;
}

/**
 * @brief  Set self test
 * @param  pObj the device pObj
 * @param  val the value of st_xl in reg CTRL5_C
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GYRO_Set_SelfTest(LSM6DSM_Object_t *pObj, uint8_t val)
{
  lsm6dsm_st_g_t reg;

  reg = (val == 0U)  ? LSM6DSM_GY_ST_DISABLE
        : (val == 1U)  ? LSM6DSM_GY_ST_POSITIVE
        : (val == 2U)  ? LSM6DSM_GY_ST_NEGATIVE
        :                LSM6DSM_GY_ST_DISABLE;


  if (lsm6dsm_gy_self_test_set(&(pObj->Ctx), reg) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM GYRO data ready bit value
 * @param  pObj the device pObj
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GYRO_Get_DRDY_Status(LSM6DSM_Object_t *pObj, uint8_t *Status)
{
  if (lsm6dsm_gy_flag_data_ready_get(&(pObj->Ctx), Status) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM GYRO initialization status
 * @param  pObj the device pObj
 * @param  Status 1 if initialized, 0 otherwise
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_GYRO_Get_Init_Status(LSM6DSM_Object_t *pObj, uint8_t *Status)
{
  if (pObj == NULL)
  {
    return LSM6DSM_ERROR;
  }

  *Status = pObj->is_initialized;

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM FIFO number of samples
 * @param  pObj the device pObj
 * @param  NumSamples number of samples
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_Get_Num_Samples(LSM6DSM_Object_t *pObj, uint16_t *NumSamples)
{
  if (lsm6dsm_fifo_data_level_get(&(pObj->Ctx), NumSamples) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM FIFO full status
 * @param  pObj the device pObj
 * @param  Status FIFO full status
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_Get_Full_Status(LSM6DSM_Object_t *pObj, uint8_t *Status)
{
  lsm6dsm_reg_t reg;

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_FIFO_STATUS2, &reg.byte, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  *Status = reg.fifo_status2.fifo_full_smart;

  return LSM6DSM_OK;
}

/**
 * @brief  Set the LSM6DSM FIFO ODR value
 * @param  pObj the device pObj
 * @param  Odr FIFO ODR value
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_Set_ODR_Value(LSM6DSM_Object_t *pObj, float Odr)
{
  lsm6dsm_odr_fifo_t new_odr;

  new_odr = (Odr <=   12.5f) ? LSM6DSM_FIFO_12Hz5
            : (Odr <=   26.0f) ? LSM6DSM_FIFO_26Hz
            : (Odr <=   52.0f) ? LSM6DSM_FIFO_52Hz
            : (Odr <=  104.0f) ? LSM6DSM_FIFO_104Hz
            : (Odr <=  208.0f) ? LSM6DSM_FIFO_208Hz
            : (Odr <=  416.0f) ? LSM6DSM_FIFO_416Hz
            : (Odr <=  833.0f) ? LSM6DSM_FIFO_833Hz
            : (Odr <= 1660.0f) ? LSM6DSM_FIFO_1k66Hz
            : (Odr <= 3330.0f) ? LSM6DSM_FIFO_3k33Hz
            :                    LSM6DSM_FIFO_6k66Hz;

  if (lsm6dsm_fifo_data_rate_set(&(pObj->Ctx), new_odr) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set the LSM6DSM FIFO full interrupt on INT1 pin
 * @param  pObj the device pObj
 * @param  Status FIFO full interrupt on INT1 pin status
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_Set_INT1_FIFO_Full(LSM6DSM_Object_t *pObj, uint8_t Status)
{
  lsm6dsm_reg_t reg;

  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_INT1_CTRL, &reg.byte, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  reg.int1_ctrl.int1_full_flag = Status;

  if (lsm6dsm_write_reg(&(pObj->Ctx), LSM6DSM_INT1_CTRL, &reg.byte, 1) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set the LSM6DSM FIFO watermark level
 * @param  pObj the device pObj
 * @param  Watermark FIFO watermark level
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_Set_Watermark_Level(LSM6DSM_Object_t *pObj, uint16_t Watermark)
{
  if (lsm6dsm_fifo_watermark_set(&(pObj->Ctx), Watermark) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set the LSM6DSM FIFO stop on watermark
 * @param  pObj the device pObj
 * @param  Status FIFO stop on watermark status
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_Set_Stop_On_Fth(LSM6DSM_Object_t *pObj, uint8_t Status)
{
  if (lsm6dsm_fifo_stop_on_wtm_set(&(pObj->Ctx), Status) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set the LSM6DSM FIFO mode
 * @param  pObj the device pObj
 * @param  Mode FIFO mode
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_Set_Mode(LSM6DSM_Object_t *pObj, uint8_t Mode)
{
  int32_t ret = LSM6DSM_OK;

  /* Verify that the passed parameter contains one of the valid values. */
  switch ((lsm6dsm_fifo_mode_t)Mode)
  {
    case LSM6DSM_BYPASS_MODE:
    case LSM6DSM_FIFO_MODE:
    case LSM6DSM_STREAM_TO_FIFO_MODE:
    case LSM6DSM_BYPASS_TO_STREAM_MODE:
    case LSM6DSM_STREAM_MODE:
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  if (ret == LSM6DSM_ERROR)
  {
    return ret;
  }

  if (lsm6dsm_fifo_mode_set(&(pObj->Ctx), (lsm6dsm_fifo_mode_t)Mode) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSM FIFO pattern
 * @param  pObj the device pObj
 * @param  Pattern FIFO pattern
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_Get_Pattern(LSM6DSM_Object_t *pObj, uint16_t *Pattern)
{
  if (lsm6dsm_fifo_pattern_get(&(pObj->Ctx), Pattern) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Get the LSM6DSM FIFO raw data
 * @param  pObj the device pObj
 * @param  Data FIFO raw data array [2]
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_Get_Data(LSM6DSM_Object_t *pObj, uint8_t *Data)
{
  if (lsm6dsm_read_reg(&(pObj->Ctx), LSM6DSM_FIFO_DATA_OUT_L, Data, 2) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set the LSM6DSM FIFO accelero decimation
 * @param  pObj the device pObj
 * @param  Decimation FIFO accelero decimation
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_ACC_Set_Decimation(LSM6DSM_Object_t *pObj, uint8_t Decimation)
{
  int32_t ret = LSM6DSM_OK;

  /* Verify that the passed parameter contains one of the valid values. */
  switch ((lsm6dsm_dec_fifo_xl_t)Decimation)
  {
    case LSM6DSM_FIFO_XL_DISABLE:
    case LSM6DSM_FIFO_XL_NO_DEC:
    case LSM6DSM_FIFO_XL_DEC_2:
    case LSM6DSM_FIFO_XL_DEC_3:
    case LSM6DSM_FIFO_XL_DEC_4:
    case LSM6DSM_FIFO_XL_DEC_8:
    case LSM6DSM_FIFO_XL_DEC_16:
    case LSM6DSM_FIFO_XL_DEC_32:
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  if (ret == LSM6DSM_ERROR)
  {
    return ret;
  }

  if (lsm6dsm_fifo_xl_batch_set(&(pObj->Ctx), (lsm6dsm_dec_fifo_xl_t)Decimation) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSM FIFO accelero single sample (16-bit data) and calculate acceleration [mg]
 * @param  pObj the device pObj
 * @param  Acceleration FIFO single accelero axis [mg]
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_ACC_Get_Axis(LSM6DSM_Object_t *pObj, int32_t *Acceleration)
{
  uint8_t data[2];
  int16_t data_raw;
  float sensitivity = 0.0f;
  float acceleration_float;

  if (LSM6DSM_FIFO_Get_Data(pObj, data) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  data_raw = ((int16_t)data[1] << 8) | data[0];

  if (LSM6DSM_ACC_GetSensitivity(pObj, &sensitivity) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  acceleration_float = (float)data_raw * sensitivity;
  *Acceleration = (int32_t)acceleration_float;

  return LSM6DSM_OK;
}

/**
 * @brief  Set the LSM6DSM FIFO gyro decimation
 * @param  pObj the device pObj
 * @param  Decimation FIFO gyro decimation
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_GYRO_Set_Decimation(LSM6DSM_Object_t *pObj, uint8_t Decimation)
{
  int32_t ret = LSM6DSM_OK;

  /* Verify that the passed parameter contains one of the valid values. */
  switch ((lsm6dsm_dec_fifo_gyro_t)Decimation)
  {
    case LSM6DSM_FIFO_GY_DISABLE:
    case LSM6DSM_FIFO_GY_NO_DEC:
    case LSM6DSM_FIFO_GY_DEC_2:
    case LSM6DSM_FIFO_GY_DEC_3:
    case LSM6DSM_FIFO_GY_DEC_4:
    case LSM6DSM_FIFO_GY_DEC_8:
    case LSM6DSM_FIFO_GY_DEC_16:
    case LSM6DSM_FIFO_GY_DEC_32:
      break;

    default:
      ret = LSM6DSM_ERROR;
      break;
  }

  if (ret == LSM6DSM_ERROR)
  {
    return ret;
  }

  if (lsm6dsm_fifo_gy_batch_set(&(pObj->Ctx), (lsm6dsm_dec_fifo_gyro_t)Decimation) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return ret;
}

/**
 * @brief  Get the LSM6DSM FIFO gyro single sample (16-bit data) and calculate angular velocity [mDPS]
 * @param  pObj the device pObj
 * @param  AngularVelocity FIFO single gyro axis [mDPS]
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LSM6DSM_FIFO_GYRO_Get_Axis(LSM6DSM_Object_t *pObj, int32_t *AngularVelocity)
{
  uint8_t data[2];
  int16_t data_raw;
  float sensitivity = 0.0f;
  float angular_velocity_float;

  if (LSM6DSM_FIFO_Get_Data(pObj, data) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  data_raw = ((int16_t)data[1] << 8) | data[0];

  if (LSM6DSM_GYRO_GetSensitivity(pObj, &sensitivity) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  angular_velocity_float = (float)data_raw * sensitivity;
  *AngularVelocity = (int32_t)angular_velocity_float;

  return LSM6DSM_OK;
}

/**
 * @}
 */

/** @defgroup LSM6DSM_Private_Functions LSM6DSM Private Functions
 * @{
 */

/**
 * @brief  Set the LSM6DSM accelerometer sensor output data rate when enabled
 * @param  pObj the device pObj
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LSM6DSM_ACC_SetOutputDataRate_When_Enabled(LSM6DSM_Object_t *pObj, float Odr)
{
  lsm6dsm_odr_xl_t new_odr;

  new_odr = (Odr <=   12.5f) ? LSM6DSM_XL_ODR_12Hz5
            : (Odr <=   26.0f) ? LSM6DSM_XL_ODR_26Hz
            : (Odr <=   52.0f) ? LSM6DSM_XL_ODR_52Hz
            : (Odr <=  104.0f) ? LSM6DSM_XL_ODR_104Hz
            : (Odr <=  208.0f) ? LSM6DSM_XL_ODR_208Hz
            : (Odr <=  416.0f) ? LSM6DSM_XL_ODR_416Hz
            : (Odr <=  833.0f) ? LSM6DSM_XL_ODR_833Hz
            : (Odr <= 1660.0f) ? LSM6DSM_XL_ODR_1k66Hz
            : (Odr <= 3330.0f) ? LSM6DSM_XL_ODR_3k33Hz
            :                    LSM6DSM_XL_ODR_6k66Hz;

  /* Output data rate selection. */
  if (lsm6dsm_xl_data_rate_set(&(pObj->Ctx), new_odr) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set the LSM6DSM accelerometer sensor output data rate when disabled
 * @param  pObj the device pObj
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LSM6DSM_ACC_SetOutputDataRate_When_Disabled(LSM6DSM_Object_t *pObj, float Odr)
{
  pObj->acc_odr = (Odr <=   12.5f) ? LSM6DSM_XL_ODR_12Hz5
                  : (Odr <=   26.0f) ? LSM6DSM_XL_ODR_26Hz
                  : (Odr <=   52.0f) ? LSM6DSM_XL_ODR_52Hz
                  : (Odr <=  104.0f) ? LSM6DSM_XL_ODR_104Hz
                  : (Odr <=  208.0f) ? LSM6DSM_XL_ODR_208Hz
                  : (Odr <=  416.0f) ? LSM6DSM_XL_ODR_416Hz
                  : (Odr <=  833.0f) ? LSM6DSM_XL_ODR_833Hz
                  : (Odr <= 1660.0f) ? LSM6DSM_XL_ODR_1k66Hz
                  : (Odr <= 3330.0f) ? LSM6DSM_XL_ODR_3k33Hz
                  :                    LSM6DSM_XL_ODR_6k66Hz;

  return LSM6DSM_OK;
}

/**
 * @brief  Set the LSM6DSM gyroscope sensor output data rate when enabled
 * @param  pObj the device pObj
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LSM6DSM_GYRO_SetOutputDataRate_When_Enabled(LSM6DSM_Object_t *pObj, float Odr)
{
  lsm6dsm_odr_g_t new_odr;

  new_odr = (Odr <=   12.5f) ? LSM6DSM_GY_ODR_12Hz5
            : (Odr <=   26.0f) ? LSM6DSM_GY_ODR_26Hz
            : (Odr <=   52.0f) ? LSM6DSM_GY_ODR_52Hz
            : (Odr <=  104.0f) ? LSM6DSM_GY_ODR_104Hz
            : (Odr <=  208.0f) ? LSM6DSM_GY_ODR_208Hz
            : (Odr <=  416.0f) ? LSM6DSM_GY_ODR_416Hz
            : (Odr <=  833.0f) ? LSM6DSM_GY_ODR_833Hz
            : (Odr <= 1660.0f) ? LSM6DSM_GY_ODR_1k66Hz
            : (Odr <= 3330.0f) ? LSM6DSM_GY_ODR_3k33Hz
            :                    LSM6DSM_GY_ODR_6k66Hz;

  /* Output data rate selection. */
  if (lsm6dsm_gy_data_rate_set(&(pObj->Ctx), new_odr) != LSM6DSM_OK)
  {
    return LSM6DSM_ERROR;
  }

  return LSM6DSM_OK;
}

/**
 * @brief  Set the LSM6DSM gyroscope sensor output data rate when disabled
 * @param  pObj the device pObj
 * @param  Odr the functional output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LSM6DSM_GYRO_SetOutputDataRate_When_Disabled(LSM6DSM_Object_t *pObj, float Odr)
{
  pObj->gyro_odr = (Odr <=   12.5f) ? LSM6DSM_GY_ODR_12Hz5
                   : (Odr <=   26.0f) ? LSM6DSM_GY_ODR_26Hz
                   : (Odr <=   52.0f) ? LSM6DSM_GY_ODR_52Hz
                   : (Odr <=  104.0f) ? LSM6DSM_GY_ODR_104Hz
                   : (Odr <=  208.0f) ? LSM6DSM_GY_ODR_208Hz
                   : (Odr <=  416.0f) ? LSM6DSM_GY_ODR_416Hz
                   : (Odr <=  833.0f) ? LSM6DSM_GY_ODR_833Hz
                   : (Odr <= 1660.0f) ? LSM6DSM_GY_ODR_1k66Hz
                   : (Odr <= 3330.0f) ? LSM6DSM_GY_ODR_3k33Hz
                   :                    LSM6DSM_GY_ODR_6k66Hz;

  return LSM6DSM_OK;
}

/**
 * @brief  Wrap Read register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LSM6DSM_Object_t *pObj = (LSM6DSM_Object_t *)Handle;

  return pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length);
}

/**
 * @brief  Wrap Write register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LSM6DSM_Object_t *pObj = (LSM6DSM_Object_t *)Handle;

  return pObj->IO.WriteReg(pObj->IO.Address, Reg, pData, Length);
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
