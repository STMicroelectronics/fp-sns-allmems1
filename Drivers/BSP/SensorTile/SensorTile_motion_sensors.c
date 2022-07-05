/**
******************************************************************************
* @file    SensorTile_motion_sensors.c
* @author  SRA - Central Labs
* @version v2.1.6
* @date    10-Feb-2022
* @brief   This file provides BSP Motion Sensors interface
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
#include "SensorTile_motion_sensors.h"

extern SPI_HandleTypeDef hbusspi2;

extern void *MotionCompObj[MOTION_INSTANCES_NBR]; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
void *MotionCompObj[MOTION_INSTANCES_NBR];

/* We define a jump table in order to get the correct index from the desired function. */
/* This table should have a size equal to the maximum value of a function plus 1.      */
static uint32_t FunctionIndex[5] = {0,0,1,1,2};
static MOTION_SENSOR_FuncDrv_t *MotionFuncDrv[MOTION_INSTANCES_NBR][MOTION_FUNCTIONS_NBR];
static MOTION_SENSOR_CommonDrv_t *MotionDrv[MOTION_INSTANCES_NBR];
static MOTION_SENSOR_Ctx_t MotionCtx[MOTION_INSTANCES_NBR];

#if (USE_MOTION_SENSOR_LSM6DSM_0 == 1)
static int32_t LSM6DSM_0_Probe(uint32_t Functions);
#endif
#if (USE_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
static int32_t LSM303AGR_ACC_0_Probe(uint32_t Functions);
#endif
#if (USE_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
static int32_t LSM303AGR_MAG_0_Probe(uint32_t Functions);
#endif

#if (USE_MOTION_SENSOR_LSM6DSM_0 == 1)
static int32_t BSP_LSM6DSM_Init(void);
static int32_t BSP_LSM6DSM_DeInit(void);
static int32_t BSP_LSM6DSM_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LSM6DSM_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
static void LSM6DSM_SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead);
static void LSM6DSM_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val);
static void LSM6DSM_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val);
#endif

#if (USE_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
static int32_t BSP_LSM303AGR_Init(void);
static int32_t BSP_LSM303AGR_DeInit(void);
static int32_t BSP_LSM303AGR_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LSM303AGR_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len);
static void LSM303AGR_SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead);
static void LSM303AGR_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val);
static void LSM303AGR_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val);
#endif

/**
  * @brief  Initializes the motion sensors
  * @param  Instance Motion sensor instance
  * @param  Functions Motion sensor functions. Could be :
  *         - MOTION_GYRO
  *         - MOTION_ACCELERO
  *         - MOTION_MAGNETO
  * @retval BSP status
  */
int32_t BSP_MOTION_SENSOR_Init(uint32_t Instance, uint32_t Functions)
{
  int32_t ret = BSP_ERROR_NONE;
  uint32_t function = MOTION_GYRO;
  uint32_t i;
  uint32_t component_functions = 0;
  MOTION_SENSOR_Capabilities_t cap;

  switch (Instance)
  {
#if (USE_MOTION_SENSOR_LSM6DSM_0 == 1)
    case LSM6DSM_0:
      if (LSM6DSM_0_Probe(Functions) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_NO_INIT;
      }
      if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], (void *)&cap) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_UNKNOWN_COMPONENT;
      }
      if (cap.Acc == 1U)
      {
        component_functions |= MOTION_ACCELERO;
      }
      if (cap.Gyro == 1U)
      {
        component_functions |= MOTION_GYRO;
      }
      if (cap.Magneto == 1U)
      {
        component_functions |= MOTION_MAGNETO;
      }
      break;
#endif
#if (USE_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
    case LSM303AGR_ACC_0:
      if (LSM303AGR_ACC_0_Probe(Functions) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_NO_INIT;
      }
      if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], (void *)&cap) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_UNKNOWN_COMPONENT;
      }
      if (cap.Acc == 1U)
      {
        component_functions |= MOTION_ACCELERO;
      }
      if (cap.Gyro == 1U)
      {
        component_functions |= MOTION_GYRO;
      }
      if (cap.Magneto == 1U)
      {
        component_functions |= MOTION_MAGNETO;
      }
      break;
#endif
#if (USE_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
    case LSM303AGR_MAG_0:
      if (LSM303AGR_MAG_0_Probe(Functions) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_NO_INIT;
      }
      if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], (void *)&cap) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_UNKNOWN_COMPONENT;
      }
      if (cap.Acc == 1U)
      {
        component_functions |= MOTION_ACCELERO;
      }
      if (cap.Gyro == 1U)
      {
        component_functions |= MOTION_GYRO;
      }
      if (cap.Magneto == 1U)
      {
        component_functions |= MOTION_MAGNETO;
      }
      break;
#endif
    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }

  for (i = 0; i < MOTION_FUNCTIONS_NBR; i++)
  {
    if (((Functions & function) == function) && ((component_functions & function) == function))
    {
      if (MotionFuncDrv[Instance][FunctionIndex[function]]->Enable(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_COMPONENT_FAILURE;
      }
    }
    function = function << 1;
  }

  return ret;
}

/**
 * @brief  Deinitialize Motion sensor
 * @param  Instance Motion sensor instance
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_DeInit(uint32_t Instance)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MotionDrv[Instance]->DeInit(MotionCompObj[Instance]) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Get motion sensor instance capabilities
 * @param  Instance Motion sensor instance
 * @param  Capabilities pointer to motion sensor capabilities
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetCapabilities(uint32_t Instance, MOTION_SENSOR_Capabilities_t *Capabilities)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MotionDrv[Instance]->GetCapabilities(MotionCompObj[Instance], Capabilities) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Get WHOAMI value
 * @param  Instance Motion sensor instance
 * @param  Id WHOAMI value
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_ReadID(uint32_t Instance, uint8_t *Id)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (MotionDrv[Instance]->ReadID(MotionCompObj[Instance], Id) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Enable Motion sensor
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_Enable(uint32_t Instance, uint32_t Function)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->Enable(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Disable Motion sensor
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_Disable(uint32_t Instance, uint32_t Function)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->Disable(MotionCompObj[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get motion sensor axes data
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Axes pointer to axes data structure
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetAxes(uint32_t Instance, uint32_t Function, BSP_MOTION_SENSOR_Axes_t *Axes)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetAxes(MotionCompObj[Instance], Axes) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get motion sensor axes raw data
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Axes pointer to axes raw data structure
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetAxesRaw(uint32_t Instance, uint32_t Function, BSP_MOTION_SENSOR_AxesRaw_t *Axes)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetAxesRaw(MotionCompObj[Instance], Axes) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get motion sensor sensitivity
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Sensitivity pointer to sensitivity read value
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetSensitivity(uint32_t Instance, uint32_t Function, float *Sensitivity)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetSensitivity(MotionCompObj[Instance],
          Sensitivity) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get motion sensor Output Data Rate
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Odr pointer to Output Data Rate read value
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, float *Odr)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetOutputDataRate(MotionCompObj[Instance], Odr) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Get motion sensor Full Scale
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Fullscale pointer to Fullscale read value
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_GetFullScale(uint32_t Instance, uint32_t Function, int32_t *Fullscale)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->GetFullScale(MotionCompObj[Instance], Fullscale) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Set motion sensor Output Data Rate
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Odr Output Data Rate value to be set
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, float Odr)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->SetOutputDataRate(MotionCompObj[Instance], Odr) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

/**
 * @brief  Set motion sensor Full Scale
 * @param  Instance Motion sensor instance
 * @param  Function Motion sensor function. Could be :
 *         - MOTION_GYRO
 *         - MOTION_ACCELERO
 *         - MOTION_MAGNETO
 * @param  Fullscale Fullscale value to be set
 * @retval BSP status
 */
int32_t BSP_MOTION_SENSOR_SetFullScale(uint32_t Instance, uint32_t Function, int32_t Fullscale)
{
  int32_t ret;

  if (Instance >= MOTION_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if ((MotionCtx[Instance].Functions & Function) == Function)
    {
      if (MotionFuncDrv[Instance][FunctionIndex[Function]]->SetFullScale(MotionCompObj[Instance], Fullscale) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
  }

  return ret;
}

#if (USE_MOTION_SENSOR_LSM6DSM_0  == 1)
/**
 * @brief  Register Bus IOs for instance 0 if LSM6DSM ID is OK
 * @param  Functions Motion sensor functions. Could be :
 *         - MOTION_GYRO and/or MOTION_ACCELERO
 * @retval BSP status
 */
static int32_t LSM6DSM_0_Probe(uint32_t Functions)
{
  LSM6DSM_IO_t            io_ctx;
  uint8_t                 id;
  static LSM6DSM_Object_t lsm6dsm_obj_0;
  LSM6DSM_Capabilities_t  cap;
  int32_t ret = BSP_ERROR_NONE;

  /* Configure the accelero driver */
  io_ctx.BusType     = LSM6DSM_SPI_3WIRES_BUS; /* SPI 4-Wires */
  io_ctx.Address     = 0x0;
  io_ctx.Init        = BSP_LSM6DSM_Init;
  io_ctx.DeInit      = BSP_LSM6DSM_DeInit;
  io_ctx.ReadReg     = BSP_LSM6DSM_ReadReg;
  io_ctx.WriteReg    = BSP_LSM6DSM_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;

  if (LSM6DSM_RegisterBusIO(&lsm6dsm_obj_0, &io_ctx) != LSM6DSM_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (LSM6DSM_ReadID(&lsm6dsm_obj_0, &id) != LSM6DSM_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (id != LSM6DSM_ID)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    (void)LSM6DSM_GetCapabilities(&lsm6dsm_obj_0, &cap);
    MotionCtx[LSM6DSM_0].Functions = ((uint32_t)cap.Gyro) | ((uint32_t)cap.Acc << 1) | ((uint32_t)cap.Magneto << 2);

    MotionCompObj[LSM6DSM_0] = &lsm6dsm_obj_0;
    /* The second cast (void *) is added to bypass Misra R11.3 rule */
    MotionDrv[LSM6DSM_0] = (MOTION_SENSOR_CommonDrv_t *)(void *)&LSM6DSM_COMMON_Driver;

    if (((Functions & MOTION_GYRO) == MOTION_GYRO) && (cap.Gyro == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[LSM6DSM_0][FunctionIndex[MOTION_GYRO]] = (MOTION_SENSOR_FuncDrv_t *)(void *)&LSM6DSM_GYRO_Driver;

      if (MotionDrv[LSM6DSM_0]->Init(MotionCompObj[LSM6DSM_0]) != LSM6DSM_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    if (((Functions & MOTION_ACCELERO) == MOTION_ACCELERO) && (cap.Acc == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[LSM6DSM_0][FunctionIndex[MOTION_ACCELERO]] = (MOTION_SENSOR_FuncDrv_t *)(void *)&LSM6DSM_ACC_Driver;

      if (MotionDrv[LSM6DSM_0]->Init(MotionCompObj[LSM6DSM_0]) != LSM6DSM_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
  }

  return ret;
}

/**
 * @brief  Initialize SPI bus for LSM6DSM
 * @retval BSP status
 */
static int32_t BSP_LSM6DSM_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;
  
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);
  
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  
  BSP_LSM6DSM_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = BSP_LSM6DSM_CS_PIN;
  HAL_GPIO_Init(BSP_LSM6DSM_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);

  if(BSP_SPI2_Init() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  DeInitialize SPI bus for LSM6DSM
 * @retval BSP status
 */
static int32_t BSP_LSM6DSM_DeInit(void)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(BSP_SPI2_DeInit() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
 * @brief  Write register by SPI bus for LSM6DSM
 * @param  Addr not used, it is only for BSP compatibility
 * @param  Reg the starting register address to be written
 * @param  pdata the pointer to the data to be written
 * @param  len the length of the data to be written
 * @retval BSP status
 */
static int32_t BSP_LSM6DSM_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  /* CS Enable */
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_RESET);

  if (BSP_SPI2_Send(&dataReg, 1) != 1)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_SPI2_Send(pdata, len) != len)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);

  return ret;
}

/**
* @brief  Read register by SPI bus for LSM6DSM
  * @param  Addr not used, it is only for BSP compatibility
  * @param  Reg the starting register address to be read
* @param  pdata the pointer to the data to be read
* @param  len the length of the data to be read
* @retval BSP status
*/
static int32_t BSP_LSM6DSM_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;
  
  /* CS Enable */
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_RESET);    
  LSM6DSM_SPI_Write(&hbusspi2, (dataReg) | 0x80);
  __HAL_SPI_DISABLE(&hbusspi2);
  SPI_1LINE_RX(&hbusspi2);
  
  if (len > 1)
  {
    LSM6DSM_SPI_Read_nBytes(&hbusspi2, (pdata), len);
  }
  else
  {
    LSM6DSM_SPI_Read(&hbusspi2, (pdata));
  }
  
  /* CS Disable */
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);    
  SPI_1LINE_TX(&hbusspi2);
  __HAL_SPI_ENABLE(&hbusspi2);
  
  return ret;
}

/**
* @brief  This function reads multiple bytes on SPI 3-wire.
* @param  xSpiHandle: SPI Handler.
* @param  val: value.
* @param  nBytesToRead: number of bytes to read.
* @retval None
*/
void LSM6DSM_SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead)
{
  /* Interrupts should be disabled during this operation */
  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);
  
  /* Transfer loop */
  while (nBytesToRead > 1U)
  {
    /* Check the RXNE flag */
    if (xSpiHandle->Instance->SR & SPI_FLAG_RXNE)
    {
      /* read the received data */
      *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
      val += sizeof(uint8_t);
      nBytesToRead--;
    }
  }
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit of the last Byte received */
  /* __DSB instruction are inserted to garantee that clock is Disabled in the right timeframe */
  
  __DSB();
  __DSB();
  __HAL_SPI_DISABLE(xSpiHandle);
  
  __enable_irq();
  
  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
* @brief  This function send a command through SPI bus.
* @param  command: command id.
* @param  uint8_t val: value.
* @retval None
*/
void LSM6DSM_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val)
{
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit */
  /* Interrupts should be disabled during this operation */
  
  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);
  __asm("dsb\n");
  __asm("dsb\n");
  __HAL_SPI_DISABLE(xSpiHandle);
  __enable_irq();
  
  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
* @brief  This function send a command through SPI bus.
* @param  command : command id.
* @param  val : value.
* @retval None
*/
void LSM6DSM_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val)
{
  /* check TXE flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);
  
  /* Write the data */
  *((__IO uint8_t*) &xSpiHandle->Instance->DR) = val;
  
  /* Wait BSY flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY);
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}


#endif

#if (USE_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
/**
 * @brief  Register Bus IOs for instance 0 if LSM303AGR ID is OK
 * @param  Functions Motion sensor functions. Could be :
 *         - MOTION_ACCELERO
 * @retval BSP status
 */
static int32_t LSM303AGR_ACC_0_Probe(uint32_t Functions)
{
  LSM303AGR_IO_t                io_ctx;
  uint8_t                       id;
  static LSM303AGR_ACC_Object_t lsm303agr_acc_obj_0;
  LSM303AGR_Capabilities_t      cap;
  int32_t ret = BSP_ERROR_NONE;

  /* Configure the accelero driver */
  io_ctx.BusType     = LSM303AGR_SPI_3WIRES_BUS; /* I2C */
  io_ctx.Address     = 0x0;
  io_ctx.Init        = BSP_LSM303AGR_Init;
  io_ctx.DeInit      = BSP_LSM303AGR_DeInit;
  io_ctx.ReadReg     = BSP_LSM303AGR_ReadReg;
  io_ctx.WriteReg    = BSP_LSM303AGR_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;

  if (LSM303AGR_ACC_RegisterBusIO(&lsm303agr_acc_obj_0, &io_ctx) != LSM303AGR_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (LSM303AGR_ACC_ReadID(&lsm303agr_acc_obj_0, &id) != LSM303AGR_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (id != (uint8_t)LSM303AGR_ID_XL)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    (void)LSM303AGR_ACC_GetCapabilities(&lsm303agr_acc_obj_0, &cap);
    MotionCtx[LSM303AGR_ACC_0].Functions = ((uint32_t)cap.Gyro) | ((uint32_t)cap.Acc << 1) | ((uint32_t)cap.Magneto << 2);

    MotionCompObj[LSM303AGR_ACC_0] = &lsm303agr_acc_obj_0;
    /* The second cast (void *) is added to bypass Misra R11.3 rule */
    MotionDrv[LSM303AGR_ACC_0] = (MOTION_SENSOR_CommonDrv_t *)(void *)&LSM303AGR_ACC_COMMON_Driver;

    if (((Functions & MOTION_ACCELERO) == MOTION_ACCELERO) && (cap.Acc == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[LSM303AGR_ACC_0][FunctionIndex[MOTION_ACCELERO]] = (MOTION_SENSOR_FuncDrv_t *)(
            void *)&LSM303AGR_ACC_Driver;

      if (MotionDrv[LSM303AGR_ACC_0]->Init(MotionCompObj[LSM303AGR_ACC_0]) != LSM303AGR_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
  }

  return ret;
}
#endif

#if (USE_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
/**
 * @brief  Register Bus IOs for instance 0 if LSM303AGR ID is OK
 * @param  Functions Motion sensor functions. Could be :
 *         - MOTION_MAGNETO
 * @retval BSP status
 */
static int32_t LSM303AGR_MAG_0_Probe(uint32_t Functions)
{
  LSM303AGR_IO_t                io_ctx;
  uint8_t                       id;
  static LSM303AGR_MAG_Object_t lsm303agr_mag_obj_0;
  LSM303AGR_Capabilities_t      cap;
  int32_t ret = BSP_ERROR_NONE;

  /* Configure the magneto driver */
  io_ctx.BusType     = LSM303AGR_SPI_3WIRES_BUS; /* I2C */
  io_ctx.Address     = 0x0;
  io_ctx.Init        = BSP_LSM303AGR_Init;
  io_ctx.DeInit      = BSP_LSM303AGR_DeInit;
  io_ctx.ReadReg     = BSP_LSM303AGR_ReadReg;
  io_ctx.WriteReg    = BSP_LSM303AGR_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;

  if (LSM303AGR_MAG_RegisterBusIO(&lsm303agr_mag_obj_0, &io_ctx) != LSM303AGR_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (LSM303AGR_MAG_ReadID(&lsm303agr_mag_obj_0, &id) != LSM303AGR_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if (id != (uint8_t)LSM303AGR_ID_MG)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    (void)LSM303AGR_MAG_GetCapabilities(&lsm303agr_mag_obj_0, &cap);
    MotionCtx[LSM303AGR_MAG_0].Functions = ((uint32_t)cap.Gyro) | ((uint32_t)cap.Acc << 1) | ((uint32_t)cap.Magneto << 2);

    MotionCompObj[LSM303AGR_MAG_0] = &lsm303agr_mag_obj_0;
    /* The second cast (void *) is added to bypass Misra R11.3 rule */
    MotionDrv[LSM303AGR_MAG_0] = (MOTION_SENSOR_CommonDrv_t *)(void *)&LSM303AGR_MAG_COMMON_Driver;

    if (((Functions & MOTION_MAGNETO) == MOTION_MAGNETO) && (cap.Magneto == 1U))
    {
      /* The second cast (void *) is added to bypass Misra R11.3 rule */
      MotionFuncDrv[LSM303AGR_MAG_0][FunctionIndex[MOTION_MAGNETO]] = (MOTION_SENSOR_FuncDrv_t *)(void *)&LSM303AGR_MAG_Driver;

      if (MotionDrv[LSM303AGR_MAG_0]->Init(MotionCompObj[LSM303AGR_MAG_0]) != LSM303AGR_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
  }

  return ret;
}



static int32_t BSP_LSM303AGR_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;
  
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_SET);
  
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  
  BSP_LSM303AGR_X_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = BSP_LSM303AGR_X_CS_PIN;
  HAL_GPIO_Init(BSP_LSM303AGR_X_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN, GPIO_PIN_SET);
  
  BSP_LSM303AGR_M_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = BSP_LSM303AGR_M_CS_PIN;
  HAL_GPIO_Init(BSP_LSM303AGR_M_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_SET);
  
  if(BSP_SPI2_Init() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }
  
  return ret;
}

static int32_t BSP_LSM303AGR_DeInit(void)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(BSP_SPI2_DeInit() == BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}


/**
 * @brief  Write register by SPI bus for LSM303AGR
 * @param  Addr not used, it is only for BSP compatibility
 * @param  Reg the starting register address to be written
 * @param  pdata the pointer to the data to be written
 * @param  len the length of the data to be written
 * @retval BSP status
 */
static int32_t BSP_LSM303AGR_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  /* CS Enable */
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_RESET);

  if (BSP_SPI2_Send(&dataReg, 1) != 1)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_SPI2_Send(pdata, len) != len)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_SET);

  return ret;
}

/**
* @brief  Read register by SPI bus for LSM303AGR
* @param  Addr not used, it is only for BSP compatibility
* @param  Reg the starting register address to be read
* @param  pdata the pointer to the data to be read
* @param  len the length of the data to be read
* @retval BSP status
*/
static int32_t BSP_LSM303AGR_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;
  
  /* CS Enable */
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_RESET);
  LSM303AGR_SPI_Write(&hbusspi2, (dataReg) | 0x80);
  __HAL_SPI_DISABLE(&hbusspi2);
  SPI_1LINE_RX(&hbusspi2);
  
  if (len > 1)
  {
    LSM303AGR_SPI_Read_nBytes(&hbusspi2, (pdata), len);
  }
  else
  {
    LSM303AGR_SPI_Read(&hbusspi2, (pdata));
  }
  
  /* CS Disable */
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_SET);
  SPI_1LINE_TX(&hbusspi2);
  __HAL_SPI_ENABLE(&hbusspi2);
  return ret;
}
                                                                           
/**
* @brief  This function reads multiple bytes on SPI 3-wire.
* @param  xSpiHandle: SPI Handler.
* @param  val: value.
* @param  nBytesToRead: number of bytes to read.
* @retval None
*/
void LSM303AGR_SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead)
{
  /* Interrupts should be disabled during this operation */
  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);
  
  /* Transfer loop */
  while (nBytesToRead > 1U)
  {
    /* Check the RXNE flag */
    if (xSpiHandle->Instance->SR & SPI_FLAG_RXNE)
    {
      /* read the received data */
      *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
      val += sizeof(uint8_t);
      nBytesToRead--;
    }
  }
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit of the last Byte received */
  /* __DSB instruction are inserted to garantee that clock is Disabled in the right timeframe */
  
  __DSB();
  __DSB();
  __HAL_SPI_DISABLE(xSpiHandle);
  
  __enable_irq();
  
  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
* @brief  This function send a command through SPI bus.
* @param  command: command id.
* @param  uint8_t val: value.
* @retval None
*/
void LSM303AGR_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val)
{
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit */
  /* Interrupts should be disabled during this operation */
  
  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);
  __asm("dsb\n");
  __asm("dsb\n");
  __HAL_SPI_DISABLE(xSpiHandle);
  __enable_irq();
  
  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
* @brief  This function send a command through SPI bus.
* @param  command : command id.
* @param  val : value.
* @retval None
*/
void LSM303AGR_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val)
{
  /* check TXE flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);
  
  /* Write the data */
  *((__IO uint8_t*) &xSpiHandle->Instance->DR) = val;
  
  /* Wait BSY flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY);
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}


#endif


