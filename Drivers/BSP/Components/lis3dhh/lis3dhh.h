/**
 ******************************************************************************
 * @file    lis3dhh.h
 * @author  MEMS Software Solutions Team
 * @brief   LIS3DHH header driver file
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LIS3DHH_H
#define LIS3DHH_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "lis3dhh_reg.h"
#include <string.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @addtogroup LIS3DHH LIS3DHH
 * @{
 */

/** @defgroup LIS3DHH_Exported_Types LIS3DHH Exported Types
 * @{
 */

typedef int32_t (*LIS3DHH_Init_Func)(void);
typedef int32_t (*LIS3DHH_DeInit_Func)(void);
typedef int32_t (*LIS3DHH_GetTick_Func)(void);
typedef int32_t (*LIS3DHH_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LIS3DHH_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef enum
{
  LIS3DHH_INT1_PIN,
  LIS3DHH_INT2_PIN,
} LIS3DHH_SensorIntPin_t;

typedef struct
{
  LIS3DHH_Init_Func         Init;
  LIS3DHH_DeInit_Func       DeInit;
  uint32_t                  BusType; /*0 means I2C, 1 means SPI 4-Wires, 2 means SPI-3-Wires */
  uint8_t                   Address;
  LIS3DHH_WriteReg_Func     WriteReg;
  LIS3DHH_ReadReg_Func      ReadReg;
  LIS3DHH_GetTick_Func      GetTick;
} LIS3DHH_IO_t;


typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} LIS3DHH_AxesRaw_t;

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} LIS3DHH_Axes_t;

typedef struct
{
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
  unsigned int SleepStatus : 1;
} LIS3DHH_Event_Status_t;

typedef struct
{
  LIS3DHH_IO_t        IO;
  lis3dhh_ctx_t      Ctx;
  uint8_t             is_initialized;
  uint8_t             acc_is_enabled;
  float               acc_odr;
} LIS3DHH_Object_t;

typedef struct
{
  uint8_t   Acc;
  uint8_t   Gyro;
  uint8_t   Magneto;
  uint8_t   LowPower;
  uint32_t  GyroMaxFS;
  uint32_t  AccMaxFS;
  uint32_t  MagMaxFS;
  float     GyroMaxOdr;
  float     AccMaxOdr;
  float     MagMaxOdr;
} LIS3DHH_Capabilities_t;

typedef struct
{
  int32_t (*Init)(LIS3DHH_Object_t *);
  int32_t (*DeInit)(LIS3DHH_Object_t *);
  int32_t (*ReadID)(LIS3DHH_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LIS3DHH_Object_t *, LIS3DHH_Capabilities_t *);
} LIS3DHH_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LIS3DHH_Object_t *);
  int32_t (*Disable)(LIS3DHH_Object_t *);
  int32_t (*GetSensitivity)(LIS3DHH_Object_t *, float *);
  int32_t (*GetOutputDataRate)(LIS3DHH_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LIS3DHH_Object_t *, float);
  int32_t (*GetFullScale)(LIS3DHH_Object_t *, int32_t *);
  int32_t (*SetFullScale)(LIS3DHH_Object_t *, int32_t);
  int32_t (*GetAxes)(LIS3DHH_Object_t *, LIS3DHH_Axes_t *);
  int32_t (*GetAxesRaw)(LIS3DHH_Object_t *, LIS3DHH_AxesRaw_t *);
} LIS3DHH_ACC_Drv_t;

/**
 * @}
 */

/** @defgroup LIS3DHH_Exported_Constants LIS3DHH Exported Constants
 * @{
 */

#define LIS3DHH_OK                       0
#define LIS3DHH_ERROR                   -1

#define LIS3DHH_I2C_BUS                 0U
#define LIS3DHH_SPI_4WIRES_BUS          1U
#define LIS3DHH_SPI_3WIRES_BUS          2U

#define LIS3DHH_ACC_SENSITIVITY         0.076f  /**< Sensitivity value [mg/digit] */


/**
 * @}
 */

/** @addtogroup LIS3DHH_Exported_Functions LIS3DHH Exported Functions
 * @{
 */

int32_t LIS3DHH_RegisterBusIO(LIS3DHH_Object_t *pObj, LIS3DHH_IO_t *pIO);
int32_t LIS3DHH_Init(LIS3DHH_Object_t *pObj);
int32_t LIS3DHH_DeInit(LIS3DHH_Object_t *pObj);
int32_t LIS3DHH_ReadID(LIS3DHH_Object_t *pObj, uint8_t *Id);
int32_t LIS3DHH_GetCapabilities(LIS3DHH_Object_t *pObj, LIS3DHH_Capabilities_t *Capabilities);

int32_t LIS3DHH_ACC_Enable(LIS3DHH_Object_t *pObj);
int32_t LIS3DHH_ACC_Disable(LIS3DHH_Object_t *pObj);
int32_t LIS3DHH_ACC_GetSensitivity(LIS3DHH_Object_t *pObj, float *Sensitivity);
int32_t LIS3DHH_ACC_GetOutputDataRate(LIS3DHH_Object_t *pObj, float *Odr);
int32_t LIS3DHH_ACC_SetOutputDataRate(LIS3DHH_Object_t *pObj, float Odr);
int32_t LIS3DHH_ACC_GetFullScale(LIS3DHH_Object_t *pObj, int32_t *FullScale);
int32_t LIS3DHH_ACC_SetFullScale(LIS3DHH_Object_t *pObj, int32_t FullScale);
int32_t LIS3DHH_ACC_GetAxesRaw(LIS3DHH_Object_t *pObj, LIS3DHH_AxesRaw_t *Value);
int32_t LIS3DHH_ACC_GetAxes(LIS3DHH_Object_t *pObj, LIS3DHH_Axes_t *Acceleration);

int32_t LIS3DHH_Read_Reg(LIS3DHH_Object_t *pObj, uint8_t reg, uint8_t *Data);
int32_t LIS3DHH_Write_Reg(LIS3DHH_Object_t *pObj, uint8_t reg, uint8_t Data);

int32_t LIS3DHH_ACC_Enable_DRDY_Interrupt(LIS3DHH_Object_t *pObj);

int32_t LIS3DHH_ACC_Set_Filter_Mode(LIS3DHH_Object_t *pObj, uint8_t filterMode);

int32_t LIS3DHH_ACC_Get_DRDY_Status(LIS3DHH_Object_t *pObj, uint8_t *Status);

int32_t LIS3DHH_FIFO_Get_Num_Samples(LIS3DHH_Object_t *pObj, uint16_t *NumSamples);
int32_t LIS3DHH_FIFO_Set_Mode(LIS3DHH_Object_t *pObj, uint8_t Mode);

/**
 * @}
 */

/** @addtogroup LIS3DHH_Exported_Variables LIS3DHH Exported Variables
 * @{
 */

extern LIS3DHH_CommonDrv_t LIS3DHH_COMMON_Driver;
extern LIS3DHH_ACC_Drv_t LIS3DHH_ACC_Driver;

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif

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
