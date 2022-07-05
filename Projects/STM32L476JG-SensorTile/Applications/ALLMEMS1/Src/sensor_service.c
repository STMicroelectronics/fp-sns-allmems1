/**
  ******************************************************************************
  * @file    sensor_service.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   Add 4 bluetooth services using vendor specific profiles.
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
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"
#include "MetaDataManager.h"
#include "sensor_service.h"
#include "console.h"
#include "HWAdvanceFeatures.h"
#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include "uuid_ble_service.h"
#include "OTA.h"

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
#include "DataLog_Manager.h"
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */ 

/* Exported variables ---------------------------------------------------------*/
int connected = FALSE;
uint8_t set_connectable = TRUE;

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
uint8_t IsSdMemsRecording= 0;
uint8_t IsSdAudioRecording= 0;
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */ 

/* SD card logging status (stop=0, start=1) */
uint8_t  SD_Card_Status= 0x00;
/* Feature mask that identify the data mens selected for recording*/
uint32_t SD_Card_FeaturesMask= 0x00000000;
/* Time range for data recording in second */
uint32_t SD_Card_StepTime= 0x00000000;

volatile uint32_t FeatureMask;

/* Imported Variables -------------------------------------------------------------*/
extern uint32_t ConnectionBleStatus;

/* Code for MotionAR integration - Start Section */
extern MAR_output_t ActivityCode;
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
extern MCP_output_t CarryPositionCode;
#endif /*  ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONFA
extern MFA_activity_t SelectedActivity;
extern uint8_t FirstFitnessActivitiesCounterValue;
#endif /* ALLMEMS1_MOTIONFA */

/* Code for MotionFX integration - Start Section */
extern BSP_MOTION_SENSOR_Axes_t MAG_Offset;
/* Code for MotionFX integration - End Section */

/* Code for MotionGR integration - Start Section */
extern MGR_output_t GestureRecognitionCode;
/* Code for MotionGR integration - End Section */

/* Code for MotionPE integration - Start Section */
extern MPE_output_t PoseEstimationCode;
extern uint8_t FirstPoseEstimationCode;
/* Code for MotionPE integration - End Section */

/* Code for MotionSD integration - Start Section */
extern MSD_output_t StandingSittingDeskCode;
extern uint8_t FirstStandingSittingDeskCode;
/* Code for MotionSD integration - End Section */

/* Code for MotionTL integration - Start Section */
extern MTL_output_t TiltMeasurement;
/* Code for MotionTL integration - End Section */

/* Code for MotionVC integration - Start Section */
extern MVC_context_t VerticalContextCode;
extern uint8_t FirstVerticalContextCode;
/* Code for MotionVC integration - End Section */

extern uint32_t ForceReMagnetoCalibration;
extern unsigned char MagnetoCalibrationDone;
  
extern uint32_t FirstConnectionConfig;

extern TIM_HandleTypeDef    TimCCHandle;
extern TIM_HandleTypeDef    TimEnvHandle;
extern TIM_HandleTypeDef    TimAudioDataHandle;

#ifdef ALLMEMS1_MOTIONFA
extern TIM_HandleTypeDef    TimFitnessActivitiesHandle;
#endif /* ALLMEMS1_MOTIONFA */

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
extern volatile uint8_t SD_CardNotPresent;
extern volatile uint32_t SD_CardLogging;
extern volatile uint8_t writeAudio_flag;

/* RTC handler declared in "main.c" file */
//extern RTC_HandleTypeDef RtcHandle;
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];

extern uint8_t bdaddr[6];
extern uint8_t NodeName[8];

extern uint32_t uhCCR4_Val;

/* Private variables ------------------------------------------------------------*/
#ifdef DISABLE_FOTA
static uint32_t FirstCommandSent= 1;
#endif /* DISABLE_FOTA */

static uint16_t HWServW2STHandle;
static uint16_t EnvironmentalCharHandle;
static uint16_t AccGyroMagCharHandle;
static uint16_t AccEventCharHandle;
static uint16_t AudioLevelCharHandle;
static uint16_t BatteryFeaturesCharHandle;

/* Code for MotionAR integration - Start Section */
static uint16_t ActivityRecCharHandle;
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
static uint16_t CarryPosRecCharHandle;
#endif /*  ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONFA
static uint16_t FitnessActivitiesCharHandle;
#endif /* ALLMEMS1_MOTIONFA */

/* Code for MotionFX integration - Start Section */
static uint16_t QuaternionsCharHandle;
static uint16_t ECompassCharHandle;
/* Code for MotionFX integration - End Section */

/* Code for MotionGR integration - Start Section */
static uint16_t GestureRecCharHandle;
/* Code for MotionGR integration - End Section */

/* Code for MotionID integration - Start Section */
static uint16_t IntensityDetCharHandle;
/* Code for MotionID integration - End Section */

/* Code for MotionTL integration - Start Section */
static uint16_t TiltMeasurementCharHandle;
static uint8_t  StartMotionTL_Calibration= 0;
static uint32_t CalibrationsPerformed= 0;
/* Code for MotionTL integration - End Section */

/* Code for MotionPE & MotionSD & MotionVC integration - Start Section */
static uint16_t MotionAlgorithmCharHandle;
static uint8_t MotionAlgorithmSelected= 0;
/* Code for MotionPE & MotionSD & MotionVC integration - End Section */

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
static uint16_t SD_CardLoggingCharHandle;
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/* Code for BlueVoice integration - Start Section */
BV_ADPCM_ProfileHandle_t BLUEVOICE_tx_handle;
BV_ADPCM_uuid_t BLUEVOICE_Char_uuids;
/* Code for BlueVoice integration - End Section */

static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle;

static uint16_t ConsoleW2STHandle;
static uint16_t TermCharHandle;
static uint16_t StdErrCharHandle;

static uint8_t LastStderrBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastTermLen;

static float UsedGyroscopeDataRate;
static float UsedAccelerometerDataRate;
static float UsedPressureDataRate;

static uint8_t  EnvironmentalCharSize = 2; /* Size for Environmental BLE characteristic */

static uint8_t  AudioUtilCharSize = 6; /* Size for AudioSync characteristic */

static uint32_t SizeOfUpdateBlueFW=0;

static uint16_t connection_handle = 0;

/* Private functions ------------------------------------------------------------*/
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
static void GAP_DisconnectionComplete_CB(void);
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length);
static uint32_t ConfigCommandParsing(uint8_t * att_data, uint8_t data_length);
static void Force_UUID_Rescan(void);

/************************************/
/* Hardware Characteristics Service */
/************************************/
static void Environmental_AttributeModified_CB(uint8_t *att_data);
static void AccGyroMag_AttributeModified_CB(uint8_t *att_data);
static void AudioLevel_AttributeModified_CB(uint8_t *att_data);
static void AccelerometerEvents_AttributeModified_CB(uint8_t *att_data);
static void BatteryFeatures_AttributeModified_CB(uint8_t *att_data);

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
static void SD_CardLogging_AttributeModified_CB(uint8_t *att_data);
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/***********************************
* Software Characteristics Service *
*        MotionXX Section          *
************************************/

/* Code for MotionAR integration - Start Section */
static void ActivityRecognition_AttributeModified_CB(uint8_t *att_data);
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
static void CarryPosition_AttributeModified_CB(uint8_t *att_data);
#endif /*  ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONFA
static void FitnessActivities_SetActivity(uint8_t *att_data);
static void FitnessActivities_AttributeModified_CB(uint8_t *att_data);
#endif /* ALLMEMS1_MOTIONFA */

/* Code for MotionFX integration - Start Section */
static void Quaternions_AttributeModified_CB(uint8_t *att_data);
static void ECompass_AttributeModified_CB(uint8_t *att_data);
/* Code for MotionFX integration - End Section */

/* Code for MotionGR integration - Start Section */
static void GestureRecognition_AttributeModified_CB(uint8_t *att_data);
/* Code for MotionGR integration - End Section */

/* Code for MotionID integration - Start Section */
static void IntensityDetection_AttributeModified_CB(uint8_t *att_data);
/* Code for MotionID integration - End Section */

/* Code for MotionPE integration - Start Section */
static void PoseEstimation_AttributeModified_CB(uint8_t MotionConnection);
/* Code for MotionPE integration - End Section */

/* Code for MotionSD integration - Start Section */
static void StandingSittingDesk_AttributeModified_CB(uint8_t MotionConnection);
/* Code for MotionSD integration - End Section */

/* Code for MotionTL integration - Start Section */
static void TiltMeasurement_AttributeModified_CB(uint8_t *att_data);
/* Code for MotionTL integration - End Section */

/* Code for MotionVC integration - Start Section */
static void VerticalContext_AttributeModified_CB(uint8_t MotionConnection);
/* Code for MotionVC integration - End Section */

/* Code for MotionPE & MotionSD & MotionVC integration - Start Section */
static void MotionAlgorithm_AttributeModified_CB(uint8_t *att_data);
static uint32_t MotionAlgorithmSet_AttributeModified_CB(uint8_t *att_data);
static void MotionAlgorithmManage_AttributeModified_CB(uint8_t AlgorithmSelected);
/* Code for MotionPE & MotionSD & MotionVC integration - End Section */

/***********************************
* Software Characteristics Service *
*          Audio Section           *
************************************/

/* Code for BlueVoice integration - Start Section */
static void BlueVoice_Audio_AttributeModified_CB(uint8_t *att_data);
static void BlueVoice_AudioSync_AttributeModified_CB(uint8_t *att_data);
/* Code for BlueVoice integration - End Section */

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
static uint32_t SD_CardLogging_CommandParsing(uint8_t * att_data, uint8_t data_length);
//static tBleStatus SD_CardLoggingStatus_Notify(uint8_t Status, uint32_t FeatureMask,uint16_t TimeStep);
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/* Private define ------------------------------------------------------------*/

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;


/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle, 
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,   
				      const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  
  if (breath > 0) {
    breath--;
  } else {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);
    
    if (ret != BLE_STATUS_SUCCESS){
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }
  
  return (ret);
}

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */


/**
 * @brief  Add the Config service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConfigW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONFIG_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3,&ConfigServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  COPY_CONFIG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ConfigCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
 
  return BLE_STATUS_SUCCESS;

fail:
  //ALLMEMS1_PRINTF("Error while adding Configuration service.\n");
  return BLE_STATUS_ERROR;
}


/**
 * @brief  Add the Console service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConsoleW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONSOLE_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*2,&ConsoleW2STHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_TERM_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, uuid, W2ST_CONSOLE_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &TermCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_STDERR_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, uuid, W2ST_CONSOLE_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &StdErrCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
     goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //ALLMEMS1_PRINTF("Error while adding Console service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update Stderr characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Stderr_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages*/
  for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastStderrBuffer,data+Offset,DataToSend);
    LastStderrLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(10);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages */
  for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastTermBuffer,data+Offset,DataToSend);
    LastTermLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
        ALLMEMS1_PRINTF("Error Updating Stdout Char\r\n");
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(20);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Stderr characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Stderr_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, LastStderrLen , LastStderrBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Term_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, LastTermLen , LastTermBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Stdout Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating Stdout Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/* Code for MotionFX integration - Start Section */
/**
 * @brief  Update quaternions characteristic value
 * @param  BSP_MOTION_SENSOR_Axes_t *data Structure containing the quaterions
 * @retval tBleStatus      Status
 */
tBleStatus Quat_Update(BSP_MOTION_SENSOR_Axes_t *data)
{
  tBleStatus ret;    

  uint8_t buff[2+ 6*SEND_N_QUATERNIONS];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));

#if SEND_N_QUATERNIONS == 1
  STORE_LE_16(buff+2,data[0].x);
  STORE_LE_16(buff+4,data[0].y);
  STORE_LE_16(buff+6,data[0].z);
#elif SEND_N_QUATERNIONS == 2
  STORE_LE_16(buff+2,data[0].x);
  STORE_LE_16(buff+4,data[0].y);
  STORE_LE_16(buff+6,data[0].z);

  STORE_LE_16(buff+8 ,data[1].x);
  STORE_LE_16(buff+10,data[1].y);
  STORE_LE_16(buff+12,data[1].z);
#elif SEND_N_QUATERNIONS == 3
  STORE_LE_16(buff+2,data[0].x);
  STORE_LE_16(buff+4,data[0].y);
  STORE_LE_16(buff+6,data[0].z);

  STORE_LE_16(buff+8 ,data[1].x);
  STORE_LE_16(buff+10,data[1].y);
  STORE_LE_16(buff+12,data[1].z);

  STORE_LE_16(buff+14,data[2].x);
  STORE_LE_16(buff+16,data[2].y);
  STORE_LE_16(buff+18,data[2].z);
#else
#error SEND_N_QUATERNIONS could be only 1,2,3
#endif
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, QuaternionsCharHandle, 0, 2+6*SEND_N_QUATERNIONS, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Quat Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating Quat Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update E-Compass characteristic value
 * @param  uint16_t Angle To Magnetic North in cents of degree [0.00 -> 259,99]
 * @retval tBleStatus      Status
 */
tBleStatus ECompass_Update(uint16_t Angle)
{
  tBleStatus ret;

  uint8_t buff[2+ 2];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_LE_16(buff+2,Angle);

  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, ECompassCharHandle, 0, 2+2,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating E-Compass Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating E-Compass Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
/**
 * @brief  Update Activity Recognition value
 * @param  MAR_output_t ActivityCode Activity Recognized
 * @retval tBleStatus      Status
 */
tBleStatus ActivityRec_Update(MAR_output_t ActivityCode)
{
  tBleStatus ret;

  uint8_t buff[2+ 1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = ActivityCode;

  ret = aci_gatt_update_char_value(HWServW2STHandle, ActivityRecCharHandle, 0, 2+1, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating ActivityRec Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating ActivityRec Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
/**
 * @brief  Update Carry Postion Recognition value
 * @param  MCP_output_t CarryPositionCode Carry Position Recognized
 * @retval tBleStatus      Status
 */
tBleStatus CarryPosRec_Update(MCP_output_t CarryPositionCode)
{
  tBleStatus ret;

  uint8_t buff[2+ 1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = CarryPositionCode;

  ret = aci_gatt_update_char_value(HWServW2STHandle, CarryPosRecCharHandle, 0, 2+1, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating CarryPosRec Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating CarryPosRec Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
#endif /*  ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONFA
/**
 * @brief  Update Fitness Activities value
 * @param  MFA_activity_t Activity
 * @param  uint32_t Counter
 * @retval tBleStatus      Status
 */
tBleStatus FitnessActivities_Update(MFA_activity_t Activity, uint32_t Counter)
{
  tBleStatus ret;

  uint8_t buff[2 + 3];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = (uint8_t)Activity;
  STORE_LE_16(buff + 3, Counter);

  ret = aci_gatt_update_char_value(HWServW2STHandle, FitnessActivitiesCharHandle, 0, 2+3, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Fitness Activities Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating Fitness Activities Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
#endif /* ALLMEMS1_MOTIONFA */

/* Code for MotionGR integration - Start Section */
/**
 * @brief  Update Gesture Recognition value
 * @param  MGR_output_t GestureCode Gesture Recognized
 * @retval tBleStatus      Status
 */
tBleStatus GestureRec_Update(MGR_output_t GestureCode)
{
  tBleStatus ret;

  uint8_t buff[2+ 1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = GestureCode;

  ret = aci_gatt_update_char_value(HWServW2STHandle, GestureRecCharHandle, 0, 2+1, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Gesture Rec Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating Gesture Rec Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
/* Code for MotionGR integration - End Section */

/* Code for MotionID integration - Start Section */
/**
 * @brief  Update Intensity Detection value
 * @param  MID_output_t MIDCode Motion Intensity Detected
 * @retval tBleStatus      Status
 */
tBleStatus IntensityDet_Update(MID_output_t MIDCode)
{
  tBleStatus ret;

  uint8_t buff[2+ 1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = MIDCode;

  ret = aci_gatt_update_char_value(HWServW2STHandle, IntensityDetCharHandle, 0, 2+1, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating IntensityDet Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating IntensityDet Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
/* Code for MotionID integration - End Section */

/* Code for MotionTL integration - Start Section */
/**
 * @brief  Update Tilt Measurement value
 * @param  
 * @retval tBleStatus      Status
 */
tBleStatus TiltMeasurement_Update(ANGLES_output_t TiltMeasurement)
{
  tBleStatus ret;

  uint8_t Buff[2 + 12];
  uint8_t BuffPos;
  
  float TempFloat;
  uint8_t *TempBuff = (uint8_t *) & TempFloat;

  STORE_LE_16(Buff  ,(HAL_GetTick()>>3));
  BuffPos= 2;
  
  TempFloat = TiltMeasurement.AnglesArray[2];
  Buff[BuffPos]= TempBuff[0];
  BuffPos++;
  Buff[BuffPos]= TempBuff[1];
  BuffPos++;
  Buff[BuffPos]= TempBuff[2];
  BuffPos++;
  Buff[BuffPos]= TempBuff[3];
  BuffPos++;
 
  TempFloat = TiltMeasurement.AnglesArray[0];
  Buff[BuffPos]= TempBuff[0];
  BuffPos++;
  Buff[BuffPos]= TempBuff[1];
  BuffPos++;
  Buff[BuffPos]= TempBuff[2];
  BuffPos++;
  Buff[BuffPos]= TempBuff[3];
  BuffPos++;
  
  TempFloat = TiltMeasurement.AnglesArray[1];
  Buff[BuffPos]= TempBuff[0];
  BuffPos++;
  Buff[BuffPos]= TempBuff[1];
  BuffPos++;
  Buff[BuffPos]= TempBuff[2];
  BuffPos++;
  Buff[BuffPos]= TempBuff[3];
  BuffPos++;

  ret = aci_gatt_update_char_value(HWServW2STHandle, TiltMeasurementCharHandle, 0, 2+12, Buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Tilt Measurement Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating Tilt Measurement Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
/* Code for MotionTL integration - End Section */

/* Code for MotionPE & MotionSD & MotionVC integration - Start Section */
/**
 * @brief  Update Motion Code value
 * @param  uint8_t MotionCode Vertical Motion Code
 * @retval tBleStatus      Status
 */
tBleStatus MotionAlgorithm_Update(uint8_t MotionCode)
{
  tBleStatus ret;

  uint8_t buff[2 + 2];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = MotionAlgorithmSelected;
  buff[3] = MotionCode;

  ret = aci_gatt_update_char_value(HWServW2STHandle, MotionAlgorithmCharHandle, 0, 2+2, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Motion Algorithm Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Motion Algorithm Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
/* Code for MotionPE & MotionSD & MotionVC integration - End Section */


/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t data)
{
  tBleStatus ret;
  uint8_t buff[2+4+1+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_BE_32(buff+2,Feature);
  buff[6] = Command;
  buff[7] = data;

  ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Configuration Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating Configuration Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Send a notification When the DS3 detects one Acceleration event
 * @param  Command to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEvent_Notify(uint16_t Command, uint8_t dimByte)
{
  tBleStatus ret= BLE_STATUS_SUCCESS;
  uint8_t buff_2[2+2];
  uint8_t buff_3[2+3];
  
  switch(dimByte)
  {
  case 2:
    STORE_LE_16(buff_2  ,(HAL_GetTick()>>3));
    STORE_LE_16(buff_2+2,Command);
    ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2+2,buff_2);
    break;
  case 3:
    STORE_LE_16(buff_3  ,(HAL_GetTick()>>3));
    buff_3[2]= 0;
    STORE_LE_16(buff_3+3,Command);
    ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2+3,buff_3);
    break;
  }
  
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating AccEvent_Notify Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating AccEvent_Notify Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint8_t  Status       - SD card logging status (stop=0, start=1)
 * @param  uint32_t FeatureMask  - Feature mask that identify the data mens selected for recording
 * @param  uint16_t TimeStep     - Time range for data recording in second
 * @retval tBleStatus Status
 */
tBleStatus SD_CardLoggingStatus_Notify(uint8_t Status, uint32_t FeatureMask,uint32_t TimeStep)
{
  tBleStatus ret;
  uint8_t buff[2+1/*Status*/+4/*Feature Mask*/+4/*Time Step*/];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = Status;
  STORE_LE_32(buff+3,FeatureMask);
  STORE_LE_32(buff+7,TimeStep);

  ret = aci_gatt_update_char_value (HWServW2STHandle, SD_CardLoggingCharHandle, 0, 2+9, buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Configuration Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating Configuration Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/**
 * @brief  Add the HW Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HW_SW_ServW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];
  
  /* Environmental */
  uint8_t max_attr_records= 4;

  if(TargetBoardFeatures.HandleGGComponent)
  {
    /* Battery Present */
    max_attr_records++;
  }
  
  /* Code for MotionAR integration - Start Section */
  max_attr_records++;
  /* Code for MotionAR integration - End Section */

  #ifdef ALLMEMS1_MOTIONCP
  max_attr_records++;
  #endif /*  ALLMEMS1_MOTIONCP */
  
  #ifdef ALLMEMS1_MOTIONFA
  max_attr_records++;
  #endif /* ALLMEMS1_MOTIONFA */
  
  /* Code for MotionFX integration - Start Section */
  /* Sensor Fusion Short */
  max_attr_records++;
  /* ECompass */
  max_attr_records++;
  /* Code for MotionFX integration - End Section */

  /* Code for MotionGR integration - Start Section */
  max_attr_records++;
  /* Code for MotionGR integration - End Section */
  
  /* Code for MotionID integration - Start Section */
  max_attr_records++;
  /* Code for MotionID integration - End Section */
 
  /* Code for MotionTL integration - Start Section */
  max_attr_records++;
  /* Code for MotionTL integration - End Section */
  
  /* Code for MotionPE & MotionSD & MotionVC integration - Start Section */
  max_attr_records++;
  /* Code for MotionPE & MotionSD & MotionVC integration - End Section */
  
  /* Code for BlueVoice integration - Start Section */
  max_attr_records+=2;
  /* Code for BlueVoice integration - End Section */
  
#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
  max_attr_records++;
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
  
  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE,
                          1+3*max_attr_records,
                          &HWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Fill the Environmental BLE Characteristc */
  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
  if(TargetBoardFeatures.NumTempSensors==2) {
    uuid[14] |= 0x05; /* Two Temperature values*/
    EnvironmentalCharSize+=2*2;
  } else if(TargetBoardFeatures.NumTempSensors==1) {
    uuid[14] |= 0x04; /* One Temperature value*/
    EnvironmentalCharSize+=2;
  }
  
  if(TargetBoardFeatures.InitHumiditySensor) {
   uuid[14] |= 0x08; /* Humidity */
   EnvironmentalCharSize+=2;
  }

  if(TargetBoardFeatures.InitPressureSensor) {
    uuid[14] |= 0x10; /* Pressure value*/
    EnvironmentalCharSize+=4;
  }

  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, EnvironmentalCharSize,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &EnvironmentalCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3*3*2,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AccGyroMagCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3, //2+2,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &AccEventCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  COPY_MIC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid,2+AUDIO_CHANNELS,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AudioLevelCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  if(TargetBoardFeatures.HandleGGComponent)
  {
    COPY_GG_W2ST_CHAR_UUID(uuid);
    ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+2+2+2+1,
                             CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                             ATTR_PERMISSION_NONE,
                             GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                             16, 0, &BatteryFeaturesCharHandle);

    if (ret != BLE_STATUS_SUCCESS) {
      goto fail;
    }
  }

  /* Code for MotionAR integration - Start Section */
  COPY_ACTIVITY_REC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &ActivityRecCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* Code for MotionAR integration - End Section */

  #ifdef ALLMEMS1_MOTIONCP
  COPY_CARRY_POSITION_REC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &CarryPosRecCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  #endif /*  ALLMEMS1_MOTIONCP */
  
  #ifdef ALLMEMS1_MOTIONFA
  COPY_FITNESS_ACTIVITIES_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3,
                           CHAR_PROP_NOTIFY | CHAR_PROP_WRITE | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &FitnessActivitiesCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  #endif /* ALLMEMS1_MOTIONFA */
  
  /* Code for MotionFX integration - Start Section */
  /* Quaternions */
  COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+6*SEND_N_QUATERNIONS,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &QuaternionsCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  /* ECompass */
  COPY_ECOMPASS_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+2,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &ECompassCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* Code for MotionFX integration - End Section */
  
  /* Code for MotionGR integration - Start Section */
  COPY_GESTURE_REC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &GestureRecCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* Code for MotionGR integration - End Section */
  
  /* Code for MotionID integration - Start Section */
  /* Motion Intensity Detection */
  COPY_INTENSITY_DET_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                         CHAR_PROP_NOTIFY,
                         ATTR_PERMISSION_NONE,
                         GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                         16, 0, &IntensityDetCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* Code for MotionID integration - End Section */
  
  /* Code for MotionTL integration - Start Section */
  COPY_TILT_MEASUREMENT_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+12,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &TiltMeasurementCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* Code for MotionTL integration - End Section */

  /* Code for MotionPE & MotionSD & MotionVC integration - Start Section */
  COPY_MOTION_ALGORITHM_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_WRITE,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &MotionAlgorithmCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* Code for MotionPE & MotionSD & MotionVC integration - End Section */
  
#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
  COPY_SD_CARD_LOGGING_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+9,
                           //CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ ,
                           CHAR_PROP_NOTIFY |CHAR_PROP_WRITE | CHAR_PROP_READ ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &SD_CardLoggingCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

  /* Code for BlueVoice integration - Start Section */
  memcpy(&BLUEVOICE_tx_handle.ServiceHandle, &HWServW2STHandle, sizeof(uint16_t));
  
  COPY_AUDIO_ADPCM_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 20,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           0, 16, 1, &BLUEVOICE_tx_handle.CharAudioHandle);
  
  COPY_AUDIO_ADPCM_SYNC_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, AudioUtilCharSize,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           0, 16, 1, &BLUEVOICE_tx_handle.CharAudioSyncHandle);
  /* Code for BlueVoice integration - End Section */

  return BLE_STATUS_SUCCESS;

fail:
  //ALLMEMS1_PRINTF("Error while adding HW's Characteristcs service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update acceleration/Gryoscope and Magneto characteristics value
 * @param  BSP_MOTION_SENSOR_Axes_t Acc Structure containing acceleration value in mg
 * @param  BSP_MOTION_SENSOR_Axes_t Gyro Structure containing Gyroscope value
 * @param  BSP_MOTION_SENSOR_Axes_t Mag Structure containing magneto value
 * @retval tBleStatus      Status
 */
tBleStatus AccGyroMag_Update(BSP_MOTION_SENSOR_Axes_t *Acc,BSP_MOTION_SENSOR_Axes_t *Gyro,BSP_MOTION_SENSOR_Axes_t *Mag)
{  
  tBleStatus ret;
  int32_t x;
  int32_t y;
  int32_t z;

  uint8_t buff[2+3*3*2];

  STORE_LE_16(buff   ,(HAL_GetTick()>>3));
  
  STORE_LE_16(buff+2 ,Acc->x);
  STORE_LE_16(buff+4 ,Acc->y);
  STORE_LE_16(buff+6 ,Acc->z);
  
  Gyro->x/=100;
  Gyro->y/=100;
  Gyro->z/=100;

  STORE_LE_16(buff+8 ,Gyro->x);
  STORE_LE_16(buff+10,Gyro->y);
  STORE_LE_16(buff+12,Gyro->z);
    
  /* Apply Magneto calibration */
  x = Mag->x - MAG_Offset.x;
  y = Mag->y - MAG_Offset.y;
  z = Mag->z - MAG_Offset.z;

  STORE_LE_16(buff+14,x);
  STORE_LE_16(buff+16,y);
  STORE_LE_16(buff+18,z);
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, AccGyroMagCharHandle, 0, 2+3*3*2, buff);
	
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Acc/Gyro/Mag Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating Acc/Gyro/Mag Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;	
}


/**
 * @brief  Update Environmental characteristic value
 * @param  int32_t Press Pressure in mbar
 * @param  uint16_t Hum humidity RH (Relative Humidity) in thenths of %
 * @param  int16_t Temp2 Temperature in tenths of degree second sensor
 * @param  int16_t Temp1 Temperature in tenths of degree first sensor
 * @retval tBleStatus   Status
 */
tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1)
{
  tBleStatus ret;
  
  uint8_t BuffPos;
  uint16_t OneTemp=0;
  
  uint8_t buff[2/*Time Stamp*/ + 4/*Press*/ + 2/*Hum*/ + 2/*Temp2*/ + 2/*Temp1*/];
  
  if( (TargetBoardFeatures.InitPressureSensor) &&
      (!TargetBoardFeatures.InitHumiditySensor) ){
        OneTemp= Temp2;
  }
  
  if( (!TargetBoardFeatures.InitPressureSensor) &&
      (TargetBoardFeatures.InitHumiditySensor) ){
        OneTemp= Temp1;
  }

  /*Time Stamp*/
  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  BuffPos=2;
  
  /*Press*/
  if(TargetBoardFeatures.InitPressureSensor) {
    STORE_LE_32(buff+BuffPos,Press);
    BuffPos+=4;
  }

  /*Hum*/
  if(TargetBoardFeatures.InitHumiditySensor) {
    STORE_LE_16(buff+BuffPos,Hum);
    BuffPos+=2;
  }

  if(TargetBoardFeatures.NumTempSensors==2) {
    STORE_LE_16(buff+BuffPos,Temp2);
    BuffPos+=2;
    
    STORE_LE_16(buff+BuffPos,Temp1);
    BuffPos+=2;
  } else if(TargetBoardFeatures.NumTempSensors==1) {
    STORE_LE_16(buff+BuffPos,OneTemp);
    BuffPos+=2;
  }

  ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, EnvironmentalCharSize,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Environmental Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating Environmental Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Microphones characteristic values
 * @param  uint16_t *Mic SNR dB Microphones array
 * @retval tBleStatus   Status
 */
tBleStatus AudioLevel_Update(uint16_t *Mic)
{  
  tBleStatus ret;
  uint16_t Counter;
  
  uint8_t buff[2+1*AUDIO_CHANNELS]; /* BlueCoin has 4 Mics */

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  for(Counter=0;Counter<AUDIO_CHANNELS;Counter++) {
    buff[2+Counter]= Mic[Counter]&0xFF;
  }
    
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, AudioLevelCharHandle, 0, 2+AUDIO_CHANNELS,buff);
  
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Mic Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;	
}

/**
 * @brief  Update Gas Gouge characteristic value
 * @param  None
 * @retval tBleStatus   Status
 */
tBleStatus BatteryReport_Update(uint32_t soc, uint32_t voltage, int32_t current)
{
  tBleStatus ret;

  uint8_t buff[2+2+2+2+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_LE_16(buff+2,soc*10);
  STORE_LE_16(buff+4,voltage);
  STORE_LE_16(buff+6,current/10);

  if(soc<15) {
    /* if it's < 15% Low Battery*/
	buff[8] = 0x00; /* Low Battery */
  } else {
    //static uint32_t PreVoltage = 0;
    static uint32_t Status     = 0x04; /* Unknown */
    //if(PreVoltage!=0)
    {
      //if(PreVoltage>voltage) 
      if(current < 0)
      {
        Status = 0x01; /* Discharging */
      } else /* if(PreVoltage<voltage) */ {
        Status = 0x03; /* Charging */
      }
    }
    buff[8] = Status;
    //PreVoltage = voltage;
  }

  ret = aci_gatt_update_char_value(HWServW2STHandle, BatteryFeaturesCharHandle, 0, 2+2+2+2+1,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Battery Report Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Error Updating Battery Report Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Puts the device in connectable mode.
 * @param  None 
 * @retval None
 */
void setConnectable(void)
{  
  char local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,NodeName[1],NodeName[2],NodeName[3],NodeName[4],NodeName[5],NodeName[6],NodeName[7]};
  uint8_t manuf_data[25];
  
//  uint8_t manuf_data[26] = {
//    2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
//    //8,0x09,NAME_BLUEMS, // Complete Name
//    8,0x09,NodeName[1],NodeName[2],NodeName[3],NodeName[4],NodeName[5],NodeName[6],NodeName[7], // Complete Name
//    13,0xFF,0x01/*SKD version */,
//    0x02,
//    0x00 /* AudioSync+AudioData */,
//    0xE0 /* ACC + Gyro + Mag + Environmental + Battery Info */,
//    0x00 /*  Hardware Events + MotionFX + SD Card Logging */,
//    0x00, /*  */
//    0x00, /* BLE MAC start */
//    0x00,
//    0x00,
//    0x00,
//    0x00,
//    0x00, /* BLE MAC stop */
//  };
  
  /* Filling Manufacter Advertise data */
  manuf_data[0 ] = 8U;
  manuf_data[1 ] = 0x09U;
  manuf_data[2 ] = NodeName[1];/* Complete Name */
  manuf_data[3 ] = NodeName[2];
  manuf_data[4 ] = NodeName[3];
  manuf_data[5 ] = NodeName[4];
  manuf_data[6 ] = NodeName[5];
  manuf_data[7 ] = NodeName[6];
  manuf_data[8 ] = NodeName[7];           
  manuf_data[9 ] = 15U;
  manuf_data[10] = 0xFFU;
  manuf_data[11] = 0x30U;/* STM Manufacter AD */
  manuf_data[12] = 0x00U;
#ifdef BLE_MANAGER_SDKV2
  manuf_data[13] = 0x02U;
#else /* BLE_MANAGER_SDKV2 */
  manuf_data[13] = 0x01U;
#endif /* BLE_MANAGER_SDKV2 */
  manuf_data[14] = 0x02U; /* Board Type */
  manuf_data[15] = 0x08U; /* Firmware ID */
  manuf_data[16] = 0x00U;
  manuf_data[17] = 0x00U;
  manuf_data[18] = 0x00U;
  
  /* BLE MAC */
  manuf_data[19] = bdaddr[5];
  manuf_data[20] = bdaddr[4];
  manuf_data[21] = bdaddr[3];
  manuf_data[22] = bdaddr[2];
  manuf_data[23] = bdaddr[1];
  manuf_data[24] = bdaddr[0];

  
#ifndef BLE_MANAGER_SDKV2
  if(TargetBoardFeatures.HandleGGComponent) {
    /* Battery Present */
    manuf_data[16] |= 0x02;
  }
  
  /* Mic */
  manuf_data[15] |= 0x04;

  if(TargetBoardFeatures.NumTempSensors==2) {
    /* Two Temperature values*/
    manuf_data[16] |= 0x05;
  } else if(TargetBoardFeatures.NumTempSensors==1) {
    /* One Temperature value*/
    manuf_data[16] |= 0x04; 
  }

  if(TargetBoardFeatures.InitHumiditySensor) {
    /* Humidity */
    manuf_data[16] |= 0x08; 
  }

  if(TargetBoardFeatures.InitPressureSensor) {
    /* Pressure value*/
    manuf_data[16] |= 0x10; 
  }
  
  /* Accelerometer Events */
  manuf_data[17] |=0x04; 
   
  /* Inertial Events */
  manuf_data[16] |=0xE0;    

  /* Code for MotionAR integration - Start Section */
  if(TargetBoardFeatures.MotionARIsInitalized) {
    manuf_data[18] |= 0x10;
  }
  /* Code for MotionAR integration - End Section */

  #ifdef ALLMEMS1_MOTIONCP
  if(TargetBoardFeatures.MotionCPIsInitalized) {
    manuf_data[18] |= 0x08;
  }
  #endif /*  ALLMEMS1_MOTIONCP */
  
  #ifdef ALLMEMS1_MOTIONFA
  if(TargetBoardFeatures.MotionFAIsInitalized) {
    manuf_data[18] |= 0x0E;
  }
  #endif /* ALLMEMS1_MOTIONFA */

  /* Code for MotionFX integration - Start Section */
  if(TargetBoardFeatures.MotionFXIsInitalized) {
    manuf_data[17] |= 0x01;
    /* ECompass */
    manuf_data[18] |= 0x40;
  }
  /* Code for MotionFX integration - End Section */
  
  /* Code for MotionGR integration - Start Section */
  if(TargetBoardFeatures.MotionGRIsInitalized) {
    manuf_data[18] |= 0x02;
  }
  /* Code for MotionGR integration - End Section */

  /* Code for MotionID integration - Start Section */
  /* Motion Intensity Detection */
  if(TargetBoardFeatures.MotionIDIsInitalized) {
    manuf_data[18] |= 0x20;
  }
  /* Code for MotionID integration - End Section */
  
  /* Code for MotionTL integration - Start Section */
  /* Tilt Measurement */
  if(TargetBoardFeatures.MotionTLIsInitalized) {
    manuf_data[18] |= 0x0D;
  }
  /* Code for MotionTL integration - End Section */
  
  /* Code for MotionPE & MotionSD & MotionVC integration - Start Section */
  if( (TargetBoardFeatures.MotionPEIsInitalized) ||
      (TargetBoardFeatures.MotionSDIsInitalized) ||
      (TargetBoardFeatures.MotionVCIsInitalized) )
  {
    manuf_data[18] |= 0x0A;
  }
  /* Code for MotionPE & MotionSD & MotionVC integration - End Section */
  
  /* Code for BlueVoice integration - Start Section */
  if(TargetBoardFeatures.AudioBVIsInitalized) {
    manuf_data[15] |= 0x48; /* AudioSync+AudioData */
  }
  /* Code for BlueVoice integration - End Section */
  
#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
    manuf_data[17] |= 0x10;
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */  
#endif /* BLE_MANAGER_SDKV2 */


  /* disable scan response */
  hci_le_set_scan_resp_data(0,NULL);
  aci_gap_set_discoverable(ADV_IND, 0, 0,
                           RANDOM_ADDR,
                           NO_WHITE_LIST_USE,
                           sizeof(local_name), local_name, 0, NULL, 0, 0);

  /* Send Advertising data */
  aci_gap_update_adv_data(25, manuf_data);
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t addr[6] Address of peer device
 * @param  uint16_t handle Connection handle
 * @retval None
 */
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  connected = TRUE;
  connection_handle = handle;

#ifdef ALLMEMS1_DEBUG_CONNECTION
  ALLMEMS1_PRINTF("\r\n>>>>>>CONNECTED %x:%x:%x:%x:%x:%x\r\n\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);
#endif /* ALLMEMS1_DEBUG_CONNECTION */

  ConnectionBleStatus=0;
  
  DisableHWFeatures();

  ForceReMagnetoCalibration     =0;
  FirstConnectionConfig  =0;
  
#ifdef DISABLE_FOTA
  FirstCommandSent       =1;
#endif /* DISABLE_FOTA */


  /* Code for BlueVoice integration - Start Section */
  if(TargetBoardFeatures.AudioBVIsInitalized){
    BluevoiceADPCM_ConnectionComplete_CB(handle); 
    {
      int ret;
      ret = aci_l2cap_connection_parameter_update_request(handle,
                                                  8 /* interval_min*/,
                                                  17 /* interval_max */,
                                                  0   /* slave_latency */,
                                                  400 /*timeout_multiplier*/);
      if (ret != BLE_STATUS_SUCCESS) {
        while (1) {
          ;
        }
      }
    }
  }
  /* Code for BlueVoice integration - End Section */  
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;

#ifdef ALLMEMS1_DEBUG_CONNECTION  
  ALLMEMS1_PRINTF("<<<<<<DISCONNECTED\r\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */

  /* Code for BlueVoice integration - Start Section */
  if(TargetBoardFeatures.AudioBVIsInitalized){
    BluevoiceADPCM_DisconnectionComplete_CB();
  }
  /* Code for BlueVoice integration - End Section */

  /* Make the device connectable again. */
  set_connectable = TRUE;

  ConnectionBleStatus=0;
  
  DisableHWFeatures();
  
//  /* Enable Accelerometer WakeUp */
//  EnableHWWakeUp();
//  
//  /* The MCU has to be woken up by the LSMDS3 which generates an interrupt on INT2 (connected to GPIOA pin 2) */
//  /* Enable MCU WakeUp on PA2 */
//  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4_HIGH);

  ForceReMagnetoCalibration     =0;
  FirstConnectionConfig  =0;

#ifdef DISABLE_FOTA
  FirstCommandSent       =1;
#endif /* DISABLE_FOTA */
  
  /* Reset for any problem during FOTA update */
  SizeOfUpdateBlueFW = 0;
  
  /************************/
  /* Stops all the Timers */
  /************************/
  
  /* Code for MotionFX integration - Start Section */
  /* Stop Timer For MotionFX */
  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
  /* Code for MotionFX  integration - End Section */

  /* Code for MotionCP & MotionGR integration - Start Section */
  /* Stop Timer For MotionCP and MotionGR */
  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
  /* Code for MotionCP & MotionGR integration - End Section */
  
  /* Code for MotionAR integration - Start Section */
  /* Stop Timer For MotionAR */
  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
  /* Code for MotionAR integration - End Section */
  
  /* Stop Timer For Acc/Gyro/Mag */
  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
  
  #ifdef ALLMEMS1_MOTIONFA
  /* Stop Timer For Fitness Activities */
  if(HAL_TIM_Base_Stop_IT(&TimFitnessActivitiesHandle) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
  #endif /* ALLMEMS1_MOTIONFA */
  
  /* Stop Timer For Environmental */
  if(HAL_TIM_Base_Stop_IT(&TimEnvHandle) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
  
  /* Stop Timer For Audio Level and Source Localisation */
  if(HAL_TIM_Base_Stop_IT(&TimAudioDataHandle) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
}

/**
 * @brief  This function is called when there is a Bluetooth Read request
 * @param  uint16_t handle Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{
  if(handle == EnvironmentalCharHandle + 1){
    /* Read Request for Pressure,Humidity, and Temperatures*/
    int32_t PressToSend;
    uint16_t HumToSend;
    int16_t Temp2ToSend,Temp1ToSend;

    /* Read all the Environmental Sensors */
    ReadEnvironmentalData(&PressToSend,&HumToSend, &Temp1ToSend,&Temp2ToSend);
    
    /* Send the Data with BLE */
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  } else if(handle == AccEventCharHandle +1) {
    /* Read Request for Acc Pedometer */
    uint16_t StepCount;
    if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
      StepCount = GetStepHWPedometer();
    } else {
      StepCount = 0;
    }
    AccEvent_Notify(StepCount, 2);
  } else if (handle == StdErrCharHandle + 1) {
    /* Send again the last packet for StdError */
    Stderr_Update_AfterRead();
  } else if (handle == TermCharHandle + 1) {
    /* Send again the last packet for Terminal */
    Term_Update_AfterRead();
    
  /* Code for MotionAR integration - Start Section */
  } else if(handle == ActivityRecCharHandle + 1){
     ActivityRec_Update(ActivityCode);
  /* Code for MotionAR integration - End Section */
     
  #ifdef ALLMEMS1_MOTIONCP
  } else if(handle == CarryPosRecCharHandle + 1){
    CarryPosRec_Update(CarryPositionCode);
  #endif /*  ALLMEMS1_MOTIONCP */
    
//  #ifdef ALLMEMS1_MOTIONFA
//  } else if(handle == FitnessActivitiesCharHandle + 1){
//    FitnessActivities_Update();
//  #endif /* ALLMEMS1_MOTIONFA */
    
  /* Code for MotionGR integration - Start Section */
  } else if(handle == GestureRecCharHandle + 1){
    GestureRec_Update(GestureRecognitionCode);
  /* Code for MotionGR integration - End Section */
  } else if(handle == BatteryFeaturesCharHandle + 1){
    
    uint32_t voltage, soc;
    int32_t current;
    uint8_t v_mode;

    /* Update Gas Gouge Status */
    BSP_GG_Task(TargetBoardFeatures.HandleGGComponent,&v_mode);

    /* Read the Gas Gouge Status */
    BSP_GG_GetVoltage(TargetBoardFeatures.HandleGGComponent, &voltage);
    BSP_GG_GetCurrent(TargetBoardFeatures.HandleGGComponent, &current);
    BSP_GG_GetSOC(TargetBoardFeatures.HandleGGComponent, &soc);
    
    BatteryReport_Update(soc, voltage, current);
#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
  } else if(handle == SD_CardLoggingCharHandle + 1){
    SD_CardLoggingStatus_Notify(SD_Card_Status, SD_Card_FeaturesMask, SD_Card_StepTime);
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
  }
 
  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application 
 * is subscribed or not to the one service
 * @param uint16_t att_handle Handle of the attribute
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length)
{
#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
  if( (!IsSdMemsRecording) && (!IsSdAudioRecording))
  {
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
    /* Code for BlueVoice integration - Start Section */
    if(TargetBoardFeatures.AudioBVIsInitalized){
      BluevoiceADPCM_AttributeModified_CB(attr_handle, data_length, att_data);
    }
    /* Code for BlueVoice integration - End Section */
    
    if(attr_handle == EnvironmentalCharHandle + 2) {
      Environmental_AttributeModified_CB(att_data);
    } else if(attr_handle == AccGyroMagCharHandle + 2) {
      AccGyroMag_AttributeModified_CB(att_data);
    } else if (attr_handle == AudioLevelCharHandle + 2) {
      AudioLevel_AttributeModified_CB(att_data);
    } else if(attr_handle == AccEventCharHandle + 2) {
      AccelerometerEvents_AttributeModified_CB(att_data);
    } else if(attr_handle == BatteryFeaturesCharHandle + 2){
       BatteryFeatures_AttributeModified_CB(att_data);
  #ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
    } else if (attr_handle == SD_CardLoggingCharHandle + 1) {
      SD_CardLogging_CommandParsing(att_data, data_length);
    } else if (attr_handle == SD_CardLoggingCharHandle + 2) {
      SD_CardLogging_AttributeModified_CB(att_data);
  #endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
    }
    /* Code for MotionAR integration - Start Section */
    else if(attr_handle == ActivityRecCharHandle + 2){
      ActivityRecognition_AttributeModified_CB(att_data);
    }
    /* Code for MotionAR integration - End Section */
    #ifdef ALLMEMS1_MOTIONCP
    else if(attr_handle == CarryPosRecCharHandle + 2){
       CarryPosition_AttributeModified_CB(att_data);
    }
    #endif /*  ALLMEMS1_MOTIONCP */
    #ifdef ALLMEMS1_MOTIONFA
    else if(attr_handle == FitnessActivitiesCharHandle + 1){
       FitnessActivities_SetActivity(att_data);
    }
    else if(attr_handle == FitnessActivitiesCharHandle + 2){
       FitnessActivities_AttributeModified_CB(att_data);
    }
    #endif /* ALLMEMS1_MOTIONFA */
    /* Code for MotionFX integration - Start Section */
    else if(attr_handle == QuaternionsCharHandle + 2){
       Quaternions_AttributeModified_CB(att_data);
    }
    else if(attr_handle == ECompassCharHandle + 2){
      ECompass_AttributeModified_CB(att_data);
    }
    /* Code for MotionFX integration - End Section */
    /* Code for MotionGR integration - Start Section */
    else if(attr_handle == GestureRecCharHandle + 2){
       GestureRecognition_AttributeModified_CB(att_data);
    }
    /* Code for MotionGR integration - End Section */
    /* Code for MotionID integration - Start Section */
    else if(attr_handle == IntensityDetCharHandle + 2){
       IntensityDetection_AttributeModified_CB(att_data);
    }
    /* Code for MotionID integration - End Section */
    /* Code for MotionTL integration - Start Section */
    else if(attr_handle == TiltMeasurementCharHandle + 2){
       TiltMeasurement_AttributeModified_CB(att_data);
    }
    /* Code for MotionTL integration - End Section */
  /* Code for MotionPE & MotionSD & MotionVC integration - Start Section */
  else if(attr_handle == MotionAlgorithmCharHandle + 2){
     MotionAlgorithm_AttributeModified_CB(att_data);
  }
  else if(attr_handle == MotionAlgorithmCharHandle + 1){
     MotionAlgorithmSet_AttributeModified_CB(att_data);
  }
  /* Code for MotionPE & MotionSD & MotionVC integration - End Section */
    /* Code for BlueVoice integration - Start Section */
    else if(attr_handle == BLUEVOICE_tx_handle.CharAudioHandle + 2){
      BlueVoice_Audio_AttributeModified_CB(att_data);
    }
    else if(attr_handle == BLUEVOICE_tx_handle.CharAudioSyncHandle + 2){
      BlueVoice_AudioSync_AttributeModified_CB(att_data);
    }
    /* Code for BlueVoice integration - End Section */ 
    else if(attr_handle == ConfigCharHandle + 2){
      if (att_data[0] == 01) {
        FirstConnectionConfig=1;
      } else if (att_data[0] == 0){
        FirstConnectionConfig=0;
      }
  #ifdef ALLMEMS1_DEBUG_CONNECTION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
        BytesToWrite =sprintf((char *)BufferToWrite,"--->Calib=%s\r\n\n", FirstConnectionConfig ? "ON" : "OFF");
       Term_Update(BufferToWrite,BytesToWrite);
      } else
        ALLMEMS1_PRINTF("--->Calib=%s\r\n\n", FirstConnectionConfig ? "ON" : "OFF");
  #endif /* ALLMEMS1_DEBUG_CONNECTION */
    } else if (attr_handle == ConfigCharHandle + 1) {
      /* Received one write command from Client on Configuration characteristc */
      ConfigCommandParsing(att_data, data_length);    
    } else if(attr_handle == StdErrCharHandle + 2){
      if (att_data[0] == 01) {
        W2ST_ON_CONNECTION(W2ST_CONNECT_STD_ERR);
      } else if (att_data[0] == 0){
        W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_ERR);
      }
    } else if(attr_handle == TermCharHandle + 2){
      if (att_data[0] == 01) {
        W2ST_ON_CONNECTION(W2ST_CONNECT_STD_TERM);
      } else if (att_data[0] == 0){
        W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_TERM);
      }
    } else if (attr_handle == TermCharHandle + 1){
      uint32_t SendBackData =1; /* By default Answer with the same message received */

      if(SizeOfUpdateBlueFW!=0) {
        /* FP-SNS-ALLMEMS1 firwmare update */
        int8_t RetValue = UpdateFWBlueMS(&SizeOfUpdateBlueFW,att_data, data_length,1);
        if(RetValue!=0) {
          MCR_FAST_TERM_UPDATE_FOR_OTA(((uint8_t *)&RetValue));
          if(RetValue==1) {
            /* if OTA checked */
            BytesToWrite =sprintf((char *)BufferToWrite,"The Board will restart in 5 seconds\r\n");
            Term_Update(BufferToWrite,BytesToWrite);
            ALLMEMS1_PRINTF("%s will restart in 5 seconds\r\n",ALLMEMS1_PACKAGENAME);
            HAL_Delay(5000);
            HAL_NVIC_SystemReset();
          }
        }
        SendBackData=0;
      } else {
        /* Received one write from Client on Terminal characteristc */
        SendBackData = DebugConsoleCommandParsing(att_data,data_length);
      }

      /* Send it back for testing */
      if(SendBackData) {
        Term_Update(att_data,data_length);
      }
    } else if (attr_handle==(0x0002+2)){
      /* Force one UUID rescan */
      Force_UUID_Rescan();
    } else {
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
        BytesToWrite =sprintf((char *)BufferToWrite, "Notification UNKNOW handle\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      } else {
        ALLMEMS1_PRINTF("Notification UNKNOW handle\r\n");
      }
    }
#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
  }
  else
  {
    if(attr_handle == TermCharHandle + 2){
      if (att_data[0] == 01) {
        W2ST_ON_CONNECTION(W2ST_CONNECT_STD_TERM);
      } else if (att_data[0] == 0){
        W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_TERM);
      }
    } else if (attr_handle == TermCharHandle + 1){
      /* By default Answer with the same message received */
      uint32_t SendBackData =1; 

      /* Received one write from Client on Terminal characteristc */
      SendBackData = DebugConsoleCommandParsing(att_data,data_length);
      
      /* Send it back for testing */
      if(SendBackData) {
        Term_Update(att_data,data_length);
      }
    } else if (attr_handle == SD_CardLoggingCharHandle + 1) {
      SD_CardLogging_CommandParsing(att_data, data_length);
    } else if (attr_handle == SD_CardLoggingCharHandle + 2) {
      SD_CardLogging_AttributeModified_CB(att_data);
    } else if(attr_handle == ConfigCharHandle + 2){
      if (att_data[0] == 01) {
        Config_Notify(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,MagnetoCalibrationDone ? 100: 0);
        Config_Notify(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,MagnetoCalibrationDone ? 100: 0);
      } else if (att_data[0] == 0){
        FirstConnectionConfig=0;
      }
    }
  }
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
}

/************************************/
/* Hardware Characteristics Service */
/************************************/

/**
 * @brief  This function is called when there is a change on the gatt attribute for Environmental
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Environmental service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void Environmental_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    W2ST_ON_CONNECTION(W2ST_CONNECT_ENV);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_Base_Start_IT(&TimEnvHandle) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
  } else if (att_data[0] == 0){
    W2ST_OFF_CONNECTION(W2ST_CONNECT_ENV);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_Base_Stop_IT(&TimEnvHandle) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->Env=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? " ON\r\n" : " OFF\r\n\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->Env=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? " ON\r\n" : " OFF\r\n\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for Acc,Gyro and Mag
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Acc,Gyro and Mag service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void AccGyroMag_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    W2ST_ON_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    }
  } else if (att_data[0] == 0) {
    W2ST_OFF_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->Acc/Gyro/Mag=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG) ? " ON\r\n" : " OFF\r\n\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->Acc/Gyro/Mag=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG) ? " ON\r\n" : " OFF\r\n\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for Audio Level
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Audio Level service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void AudioLevel_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    int32_t Count;
    
    W2ST_ON_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL);
      
    InitMics(AUDIO_SAMPLING_FREQUENCY, AUDIO_VOLUME_INPUT, PCM_AUDIO_IN_SAMPLES);
    
    for(Count=0;Count<AUDIO_CHANNELS;Count++) {
      RMS_Ch[Count]=0;
      DBNOISE_Value_Old_Ch[Count] =0;
    }
    
    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_Base_Start_IT(&TimAudioDataHandle) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
  } else if (att_data[0] == 0) {
    W2ST_OFF_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL);

    DeInitMics();

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_Base_Stop_IT(&TimAudioDataHandle) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite = sprintf((char *)BufferToWrite,"--->dB Noise AudioLevel=%s", (W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL)   ? " ON\r\n" : " OFF\r\n\n") );
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->dB Noise AudioLevel=%s", (W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL)   ? " ON\r\n" : " OFF\r\n\n"));
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for Accelerometer Events
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Accelerometer Events service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void AccelerometerEvents_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    W2ST_ON_CONNECTION(W2ST_CONNECT_ACC_EVENT);
    EnableHWMultipleEvents();
    ResetHWPedometer();
    Config_Notify(FEATURE_MASK_ACC_EVENTS,'m',1);
  } else if (att_data[0] == 0) {
    W2ST_OFF_CONNECTION(W2ST_CONNECT_ACC_EVENT);
    DisableHWMultipleEvents();
    }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->AccEvent=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_EVENT) ? " ON\r\n" : " OFF\r\n\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->AccEvent=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_EVENT) ? " ON\r\n" : " OFF\r\n\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for Battery Features
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Battery Features service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void BatteryFeatures_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    W2ST_ON_CONNECTION(W2ST_CONNECT_GG_EVENT);
   /* Start the TIM Base generation in interrupt mode */
   if(HAL_TIM_Base_Start_IT(&TimEnvHandle) != HAL_OK){
     /* Starting Error */
     Error_Handler();
   }
  } else if (att_data[0] == 0){
    W2ST_OFF_CONNECTION(W2ST_CONNECT_GG_EVENT);
   /* Stop the TIM Base generation in interrupt mode */
   if(HAL_TIM_Base_Stop_IT(&TimEnvHandle) != HAL_OK){
     /* Stopping Error */
     Error_Handler();
   }
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->Battery Report=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT) ? " ON\r\n" : " OFF\r\n\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->Battery Report=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT) ? " ON\r\n" : " OFF\r\n\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}


#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
/**
 * @brief  This function is called when there is a change on the gatt attribute for SD Card Logging
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Accelerometer Events service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void SD_CardLogging_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    W2ST_ON_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING);
    SD_CardLoggingStatus_Notify(SD_Card_Status, SD_Card_FeaturesMask, SD_Card_StepTime);
  } else if (att_data[0] == 0) {
    W2ST_OFF_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING);
  }
  
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite = sprintf((char *)BufferToWrite,"--->SD_CardLogging=%s", (W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING)   ? " ON\r\n" : " OFF\r\n\n") );
   Term_Update(BufferToWrite,BytesToWrite);
  }else {
    ALLMEMS1_PRINTF("--->SD_CardLogging=%s", (W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD_CARD_LOGGING)   ? " ON\r\n" : " OFF\r\n\n"));
  }
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/***********************************
* Software Characteristics Service *
*        MotionXX Section          *
************************************/

/* Code for MotionAR integration - Start Section */
/**
 * @brief  This function is called when there is a change on the gatt attribute for Activity Recognition
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Activity Recognition service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void ActivityRecognition_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    Set4GAccelerometerFullScale();

    W2ST_ON_CONNECTION(W2ST_CONNECT_AR);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + DEFAULT_uhCCR3_Val));
    }
  } else if (att_data[0] == 0){

    Set2GAccelerometerFullScale();

    W2ST_OFF_CONNECTION(W2ST_CONNECT_AR);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->ActRec=%s",  W2ST_CHECK_CONNECTION(W2ST_CONNECT_AR) ? " ON\r\n" : " OFF\r\n\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->ActRec=%s",  W2ST_CHECK_CONNECTION(W2ST_CONNECT_AR) ? " ON\r\n" : " OFF\r\n\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
/**
 * @brief  This function is called when there is a change on the gatt attribute for Carry Position
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Carry Position service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void CarryPosition_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    Set4GAccelerometerFullScale();

    W2ST_ON_CONNECTION(W2ST_CONNECT_CP);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
    /* Set the new Capture compare value */
    {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + DEFAULT_uhCCR2_Val));
    }
  } else if (att_data[0] == 0){

    Set2GAccelerometerFullScale();

    W2ST_OFF_CONNECTION(W2ST_CONNECT_CP);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->CarryPosRec=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_CP) ? " ON\r\n" : " OFF\r\n\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->CarryPosRec=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_CP) ? " ON\r\n" : " OFF\r\n\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}
#endif /*  ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONFA
/**
 * @brief  This function is called when there is a change on the gatt attribute for Fitness Activities
 * With this function it's possible to set the selected activity
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void FitnessActivities_SetActivity(uint8_t *att_data)
{
    SelectedActivity= (MFA_activity_t) att_data[0];
    MotionFA_manager_reset_counter();
    MotionFA_manager_set_activity(SelectedActivity);
    
    FirstFitnessActivitiesCounterValue= 1;
    
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->FitnessActivities= %d\r\n", SelectedActivity);
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->SelectedActivities= %d\r\n", SelectedActivity);
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for Fitness Activities
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Fitness Activities service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void FitnessActivities_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    
    /* Get ODR accelerometer default Value */
    BSP_MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO,&UsedAccelerometerDataRate);
    /* Set ODR accelerometer: >= 25 Hz*/
    BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, 25.0f);
    
    /* Get ODR gyroscope default Value */
    BSP_MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE, MOTION_GYRO,&UsedGyroscopeDataRate);
    /* Set ODR gyroscope: >= 25 Hz*/
    BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_GYRO, 25.0f);
    
    /* Get ODR pressure sensor default Value */
    BSP_ENV_SENSOR_GetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, &UsedPressureDataRate);
    /* Set ODR pressure sensor: >= 25 Hz*/
    BSP_ENV_SENSOR_SetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, 25.0f);
    
    W2ST_ON_CONNECTION(W2ST_CONNECT_FA);
    
    MotionFA_manager_reset_counter();
    FirstFitnessActivitiesCounterValue= 1;

      /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_Base_Start_IT(&TimFitnessActivitiesHandle) != HAL_OK){
        /* Starting Error */
      Error_Handler();
    }
  } else if (att_data[0] == 0){
    
    /* Set default ODR accelerometer */
    BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, UsedAccelerometerDataRate);
    
    /* Set default ODR gyroscope */
    BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_GYRO, UsedGyroscopeDataRate);

    /* Set default ODR pressure sensor */
    BSP_ENV_SENSOR_SetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, UsedPressureDataRate);
    
    W2ST_OFF_CONNECTION(W2ST_CONNECT_FA);
    FirstFitnessActivitiesCounterValue= 0;

      /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_Base_Stop_IT(&TimFitnessActivitiesHandle) != HAL_OK){
        /* Stopping Error */
      Error_Handler();
    }
  }
    
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->FitnessActivities=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_FA) ? " ON\r\n" : " OFF\r\n\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->FitnessActivities=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_FA) ? " ON\r\n" : " OFF\r\n\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}
#endif /* ALLMEMS1_MOTIONFA */

/* Code for MotionFX integration - Start Section */
/**
 * @brief  This function is called when there is a change on the gatt attribute for quaternions
 * With this function it's possible to understand if one application 
 * is subscribed or not to the quaternions service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void Quaternions_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    W2ST_ON_CONNECTION(W2ST_CONNECT_QUAT);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + DEFAULT_uhCCR1_Val));
    }
  } else if (att_data[0] == 0){
    W2ST_OFF_CONNECTION(W2ST_CONNECT_QUAT);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->Quater=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_QUAT) ? " ON\r\n" : " OFF\r\n\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->Quater=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_QUAT) ? " ON\r\n" : " OFF\r\n\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for ECompass
 * With this function it's possible to understand if one application 
 * is subscribed or not to the ECompass service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void ECompass_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    W2ST_ON_CONNECTION(W2ST_CONNECT_EC);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + DEFAULT_uhCCR1_Val));
    }
  } else if (att_data[0] == 0){
    W2ST_OFF_CONNECTION(W2ST_CONNECT_EC);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->ECompass=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_EC) ? " ON\r\n" : " OFF\r\n\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->ECompass=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_EC) ? " ON\r\n" : " OFF\r\n\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}
/* Code for MotionFX integration - End Section */

/* Code for MotionGR integration - Start Section */
/**
 * @brief  This function is called when there is a change on the gatt attribute for Gesture Recognition
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Gesture Recognition service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void GestureRecognition_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {

    Set4GAccelerometerFullScale();

    W2ST_ON_CONNECTION(W2ST_CONNECT_GR);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
    /* Set the new Capture compare value */
    {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + DEFAULT_uhCCR2_Val));
    }
  } else if (att_data[0] == 0){

    Set2GAccelerometerFullScale();

    W2ST_OFF_CONNECTION(W2ST_CONNECT_GR);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->GestureRec=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_GR) ? " ON\r\n" : " OFF\r\n\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->GestureRec=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_GR) ? " ON\r\n" : " OFF\r\n\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}
/* Code for MotionGR integration - End Section */

/* Code for MotionID integration - Start Section */
/**
 * @brief  This function is called when there is a change on the gatt attribute for Intensity Detection
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Intensity Detection service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void IntensityDetection_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    
    Set4GAccelerometerFullScale();

    W2ST_ON_CONNECTION(W2ST_CONNECT_ID);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
    /* Set the new Capture compare value */
    {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + DEFAULT_uhCCR3_Val));
    }
  } else if (att_data[0] == 0){

    Set2GAccelerometerFullScale();

    W2ST_OFF_CONNECTION(W2ST_CONNECT_ID);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->IntDet=%s",  W2ST_CHECK_CONNECTION(W2ST_CONNECT_ID) ? "ON\r\n" : "OFF\r\n\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->IntDet=%s",  W2ST_CHECK_CONNECTION(W2ST_CONNECT_ID) ? "ON\r\n" : "OFF\r\n\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}
/* Code for MotionID integration - End Section */

/* Code for MotionPE integration - Start Section */
/**
 * @brief  This function is called when there is a change on the gatt attribute for Pose Estimation
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Pose Estimation service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void PoseEstimation_AttributeModified_CB(uint8_t MotionConnection)
{
  if (MotionConnection == 01) {
    /* Get ODR accelerometer default Value */
    BSP_MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,&UsedAccelerometerDataRate);
    /* Set ODR accelerometer: >= 16 Hz*/
    BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, 16.0f);
    /* Set FS accelerometer: = <-4g, 4g> */
    Set4GAccelerometerFullScale();

    FirstPoseEstimationCode= 1;
    MotionPE_ResetLib();

    W2ST_ON_CONNECTION(W2ST_CONNECT_PE);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
    /* Set the new Capture compare value */
    {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + DEFAULT_uhCCR3_Val));
    }
  } else if (MotionConnection == 0){

    /* Set default ODR accelerometer */
    BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, UsedAccelerometerDataRate);
    /* Set default FS accelerometer: = <-2g, 2g> */
    Set2GAccelerometerFullScale();

    FirstPoseEstimationCode= 0;

    W2ST_OFF_CONNECTION(W2ST_CONNECT_PE);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->PoseEstimation=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_PE) ? " ON\r\n" : " OFF\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->PoseEstimation=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_PE) ? " ON\r\n" : " OFF\r\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}
/* Code for MotionPE integration - End Section */
   
/* Code for MotionSD integration - Start Section */
/**
 * @brief  This function is called when there is a change on the gatt attribute for Standing Sitting Desk
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Standing Sitting Desk service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void StandingSittingDesk_AttributeModified_CB(uint8_t MotionConnection)
{
  if (MotionConnection == 01) {
    /* Get ODR accelerometer default Value */
    BSP_MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,&UsedAccelerometerDataRate);
    /* Set ODR accelerometer: >= 25 Hz*/
    BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, 25.0f);

    /* Get ODR pressure sensor default Value */
    BSP_ENV_SENSOR_GetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, &UsedPressureDataRate);
    /* Set ODR pressure sensor: >= 25 Hz*/
    BSP_ENV_SENSOR_SetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, 25.0f);

    FirstStandingSittingDeskCode= 1;
    MotionSD_Reset();

    W2ST_ON_CONNECTION(W2ST_CONNECT_SD);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
    /* Set the new Capture compare value */
    {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + DEFAULT_uhCCR2_Val));
    }
  } else if (MotionConnection == 0){
    /* Set default ODR accelerometer */
    BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, UsedAccelerometerDataRate);

    /* Set default ODR pressure sensor */
    BSP_ENV_SENSOR_SetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, UsedPressureDataRate);

    FirstStandingSittingDeskCode= 0;

    W2ST_OFF_CONNECTION(W2ST_CONNECT_SD);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->StandingSittingDesk=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD) ? " ON\r\n" : " OFF\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->StandingSittingDesk=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD) ? " ON\r\n" : " OFF\r\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}
/* Code for MotionSD integration - End Section */

/* Code for MotionTL integration - Start Section */
/**
 * @brief  This function is called when there is a change on the gatt attribute for Tilt Measurement
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Tilt Measurement service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void TiltMeasurement_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    /* Get ODR accelerometer default Value */
    BSP_MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,&UsedAccelerometerDataRate);
    /* Set ODR accelerometer: >= 50 Hz*/
    BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, 50.0f);
    /* Set FS accelerometer: = <-4g, 4g> */
    Set4GAccelerometerFullScale();

    W2ST_ON_CONNECTION(W2ST_CONNECT_TL);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
    /* Set the new Capture compare value */
    {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + DEFAULT_uhCCR2_Val));
    }
  } else if (att_data[0] == 0){
    /* Set default ODR accelerometer value */
    BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, UsedAccelerometerDataRate);

    /* Set default FS accelerometer: = <-2g, 2g> */
    Set2GAccelerometerFullScale();

    W2ST_OFF_CONNECTION(W2ST_CONNECT_TL);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->TiltMeasurement=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_TL) ? " ON\r\n" : " OFF\r\n\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->TiltMeasurement=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_TL) ? " ON\r\n" : " OFF\r\n\r\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}
/* Code for MotionTL integration - End Section */
   
/* Code for MotionVC integration - Start Section */
/**
 * @brief  This function is called when there is a change on the gatt attribute for Vertical Context
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Vertical Context service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void VerticalContext_AttributeModified_CB(uint8_t MotionConnection)
{
  if (MotionConnection == 01) {
    /* Get ODR accelerometer default Value */
    BSP_MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,&UsedAccelerometerDataRate);
    /* Set ODR accelerometer: >= 50 Hz*/
    BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, 50.0f);
    /* Set FS accelerometer: = <-4g, 4g> */
    Set4GAccelerometerFullScale();

    FirstVerticalContextCode= 1;

    /* Get ODR pressure sensor default Value */
    BSP_ENV_SENSOR_GetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, &UsedPressureDataRate);
    /* Set ODR pressure sensor: >= 25 Hz*/
    BSP_ENV_SENSOR_SetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, 25.0f);

    W2ST_ON_CONNECTION(W2ST_CONNECT_VC);

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }
    /* Set the new Capture compare value */
    {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + DEFAULT_uhCCR2_Val));
    }
  } else if (MotionConnection == 0){
   
    /* Set default ODR accelerometer value */
    BSP_MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, UsedAccelerometerDataRate);

    /* Set default FS accelerometer: = <-2g, 2g> */
    Set2GAccelerometerFullScale();

    FirstVerticalContextCode= 0;

    /* Set default ODR pressure sensor value*/
    BSP_ENV_SENSOR_SetOutputDataRate(PRESSURE_INSTANCE, ENV_PRESSURE, UsedPressureDataRate);

    W2ST_OFF_CONNECTION(W2ST_CONNECT_VC);

    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->VerticalContext=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_VC) ? " ON\r\n" : " OFF\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->VerticalContext=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_VC) ? " ON\r\n" : " OFF\r\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}
/* Code for MotionVC integration - End Section */
       
/* Code for MotionPE & MotionSD & MotionVC integration - Start Section */
/**
 * @brief  This function is called when there is a change on the gatt attribute for Motion Algorithm
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Motion Algorithm service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void MotionAlgorithm_AttributeModified_CB(uint8_t *att_data)
{
  if (att_data[0] == 01) {
    W2ST_ON_CONNECTION(W2ST_MOTION_ALGORITHM);
  } else if (att_data[0] == 0){
    W2ST_OFF_CONNECTION(W2ST_MOTION_ALGORITHM);
  }
  
  if(!W2ST_CHECK_CONNECTION(W2ST_MOTION_ALGORITHM))
    MotionAlgorithmManage_AttributeModified_CB(MotionAlgorithmSelected);
  
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->MotionAlgorithm= %s", W2ST_CHECK_CONNECTION(W2ST_MOTION_ALGORITHM) ? " ON\r\n" : " OFF\r\n\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->MotionAlgorithm=%s", W2ST_CHECK_CONNECTION(W2ST_MOTION_ALGORITHM) ? " ON\r\n" : " OFF\r\n\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
  
  if(W2ST_CHECK_CONNECTION(W2ST_MOTION_ALGORITHM))
    MotionAlgorithmManage_AttributeModified_CB(MotionAlgorithmSelected);  
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for Motion Algorithm
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Motion Algorithm service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static uint32_t MotionAlgorithmSet_AttributeModified_CB(uint8_t *att_data)
{
  uint32_t SendItBack = 1;
  
  MotionAlgorithmSelected= att_data[0];

  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_PE))
    PoseEstimation_AttributeModified_CB(0);
  
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SD))
    StandingSittingDesk_AttributeModified_CB(0);
  
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_VC))
    VerticalContext_AttributeModified_CB(0);
  
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->MotionAlgorithmSelected= %d\r\n", MotionAlgorithmSelected);
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->MotionAlgorithmSelected= %d\r\n", MotionAlgorithmSelected);
#endif /* ALLMEMS1_DEBUG_CONNECTION */
  
  MotionAlgorithmManage_AttributeModified_CB(MotionAlgorithmSelected);
  
  return SendItBack;
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for Motion Algorithm
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Motion Algorithm service
 * @param uint8_t AlgorithmSelected
 * @retval None
 */
static void MotionAlgorithmManage_AttributeModified_CB(uint8_t AlgorithmSelected)
{
  uint8_t MotionConnection= W2ST_CHECK_CONNECTION(W2ST_MOTION_ALGORITHM);
  
  switch(AlgorithmSelected)
  {
  case 1:
    PoseEstimation_AttributeModified_CB(MotionConnection);
    break;
  case 2:
    StandingSittingDesk_AttributeModified_CB(MotionConnection);
    break;
  case 3:
    VerticalContext_AttributeModified_CB(MotionConnection);
    break;
  }
}
/* Code for MotionPE & MotionSD & MotionVC integration - End Section */

/***********************************
* Software Characteristics Service *
*          Audio Section           *
************************************/

/* Code for BlueVoice integration - Start Section */
/**
 * @brief  This function is called when there is a change on the gatt attribute for Audio Blue Voice
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Audio Blue Voice service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void BlueVoice_Audio_AttributeModified_CB(uint8_t *att_data)
{
  if(att_data[0] == 01){
    InitMics(BV_AUDIO_SAMPLING_FREQUENCY, BV_AUDIO_VOLUME_VALUE, BV_PCM_AUDIO_IN_SAMPLES);
    W2ST_ON_CONNECTION(W2ST_CONNECT_BV_AUDIO);
  } else if (att_data[0] == 0){
    LedOffTargetPlatform();
    DeInitMics();
    W2ST_OFF_CONNECTION(W2ST_CONNECT_BV_AUDIO);
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->BV_ADPCM=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_AUDIO) ? " ON\r\n" : " OFF\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->BV_ADPCM=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_AUDIO) ? " ON\r\n" : " OFF\r\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}

/**
 * @brief  This function is called when there is a change on the gatt attribute for Audio Sync Blue Voice
 * With this function it's possible to understand if one application 
 * is subscribed or not to the Audio Sync Blue Voice service
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void BlueVoice_AudioSync_AttributeModified_CB(uint8_t *att_data)
{
  if(att_data[0] == 01){      
    W2ST_ON_CONNECTION(W2ST_CONNECT_BV_SYNC);
  } else if (att_data[0] == 0){
    W2ST_OFF_CONNECTION(W2ST_CONNECT_BV_SYNC);
  }
#ifdef ALLMEMS1_DEBUG_CONNECTION   
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
    BytesToWrite =sprintf((char *)BufferToWrite,"--->BV_ADPCM_Sync=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_SYNC) ? " ON\r\n" : " OFF\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  } else
    ALLMEMS1_PRINTF("--->BV_ADPCM_Sync=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_SYNC) ? " ON\r\n" : " OFF\r\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */
}
/* Code for BlueVoice integration - End Section */

/**
 * @brief  This function makes the parsing of the Debug Console Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t SendBackData = 1;

    /* Help Command */
    if(!strncmp("help",(char *)(att_data),4)) {
      /* Print Legend */
      SendBackData=0;

      BytesToWrite =sprintf((char *)BufferToWrite,"Command:\r\n"
         "info -> System Info\r\n"
         "powerstatus-> Battery Status [%% mV mA]\r\n"
         "versionFw  -> FW Version\r\n"
         "versionBle -> Ble Version\r\n"
           );
      Term_Update(BufferToWrite,BytesToWrite);
      
      BytesToWrite =sprintf((char *)BufferToWrite,
         "TLcalibstart  -> Start MotionTL Calibration\r\n"
           );
      Term_Update(BufferToWrite,BytesToWrite);
      
      BytesToWrite =sprintf((char *)BufferToWrite,
         "TLcalibstop  -> Stop MotionTL Calibration\r\n"
           );
      Term_Update(BufferToWrite,BytesToWrite);
      
      BytesToWrite =sprintf((char *)BufferToWrite,
         "setName xxxxxxx     -> Set the node name (Max 7 characters)\r\n"
#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
         "setDate wd/dd/mm/aa -> Set date\r\n"
         "setTime hh:mm:ss    -> Set time\r\n"
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */ 
           );
      Term_Update(BufferToWrite,BytesToWrite);
      
#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
      BytesToWrite =sprintf((char *)BufferToWrite,
           "start -> Start SD card data MEMS log\r\n"
           "stop  -> Stop  SD card data MEMS log\r\n"
           "AudioStart -> Start SD card data Audio log\r\n"
           "AudioStop  -> Stop  SD card data Audio log\r\n"
             );
      Term_Update(BufferToWrite,BytesToWrite);
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */   
    } else if(!strncmp("versionFw",(char *)(att_data),9)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"%s_%s_%c.%c.%c\r\n",
#ifdef STM32F401xE
                            "F401"
#elif STM32F446xx
                            "F446"
#elif STM32L476xx
                            "L476"
#else
#error "Undefined STM32 processor type"
#endif
                            ,ALLMEMS1_PACKAGENAME,
                            ALLMEMS1_VERSION_MAJOR,
                            ALLMEMS1_VERSION_MINOR,
                            ALLMEMS1_VERSION_PATCH);
#ifdef DISABLE_FOTA
      if(FirstCommandSent)
      {
        FirstCommandSent= 0;
#endif /* DISABLE_FOTA */
        Term_Update(BufferToWrite,BytesToWrite);
        SendBackData=0;
#ifdef DISABLE_FOTA
      }
      else
        SendBackData=1;
#endif /* DISABLE_FOTA */
    } else if(!strncmp("powerstatus",(char *)(att_data),11)) {
      SendBackData=0;
      if(TargetBoardFeatures.HandleGGComponent) {
        uint32_t voltage, soc;
        uint8_t v_mode;
        int32_t current;
        /* Update Gas Gouge Status */
        BSP_GG_Task(TargetBoardFeatures.HandleGGComponent,&v_mode);

        /* Read the Gas Gouge Status */
        BSP_GG_GetVoltage(TargetBoardFeatures.HandleGGComponent, &voltage);
        BSP_GG_GetSOC(TargetBoardFeatures.HandleGGComponent, &soc);
        BSP_GG_GetCurrent(TargetBoardFeatures.HandleGGComponent, &current);

        BytesToWrite =sprintf((char *)BufferToWrite,"Battery %ld%% %ld mV Current=%ld mA\r\n",soc,voltage,current);
      } else {
        BytesToWrite =sprintf((char *)BufferToWrite,"Battery not present\r\n");
      }
      Term_Update(BufferToWrite,BytesToWrite);
    } else if(!strncmp("info",(char *)(att_data),4)) {
      SendBackData=0;
      
      BytesToWrite =sprintf((char *)BufferToWrite,"\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
         "\tSTM32476JG-SensorTile board"
         "\r\n",
         ALLMEMS1_PACKAGENAME,
         ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH);
      Term_Update(BufferToWrite,BytesToWrite);

      BytesToWrite =sprintf((char *)BufferToWrite,"\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n",
#elif defined (__CC_ARM)
        " (KEIL)\r\n",
#elif defined (__GNUC__)
        " (STM32CubeIDE)\r\n",
#endif
          HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
           __DATE__,__TIME__);
      Term_Update(BufferToWrite,BytesToWrite);
      
#ifndef DISABLE_FOTA
    }  if(!strncmp("upgradeFw",(char *)(att_data),9)) {
      uint32_t uwCRCValue;
      uint8_t *PointerByte = (uint8_t*) &SizeOfUpdateBlueFW;

      SizeOfUpdateBlueFW=atoi((char *)(att_data+9));
      PointerByte[0]=att_data[ 9];
      PointerByte[1]=att_data[10];
      PointerByte[2]=att_data[11];
      PointerByte[3]=att_data[12];

      /* Check the Maximum Possible OTA size */
      if(SizeOfUpdateBlueFW>OTA_MAX_PROG_SIZE) {
        ALLMEMS1_PRINTF("OTA %s SIZE=%ld > %d Max Allowed\r\n",ALLMEMS1_PACKAGENAME,SizeOfUpdateBlueFW, OTA_MAX_PROG_SIZE);
        /* Answer with a wrong CRC value for signaling the problem to BlueMS application */
        PointerByte[0]= att_data[13];
        PointerByte[1]=(att_data[14]!=0) ? 0 : 1;/* In order to be sure to have a wrong CRC */
        PointerByte[2]= att_data[15];
        PointerByte[3]= att_data[16];
        BytesToWrite = 4;
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        PointerByte = (uint8_t*) &uwCRCValue;
        PointerByte[0]=att_data[13];
        PointerByte[1]=att_data[14];
        PointerByte[2]=att_data[15];
        PointerByte[3]=att_data[16];

        ALLMEMS1_PRINTF("OTA %s SIZE=%ld uwCRCValue=%lx\r\n",ALLMEMS1_PACKAGENAME,SizeOfUpdateBlueFW,uwCRCValue);
	  
        /* Reset the Flash */
        StartUpdateFWBlueMS(SizeOfUpdateBlueFW,uwCRCValue);

        /* Reduce the connection interval */
        {
          int ret = aci_l2cap_connection_parameter_update_request(connection_handle,
                                                        10 /* interval_min*/,
                                                        10 /* interval_max */,
                                                        0   /* slave_latency */,
                                                        400 /*timeout_multiplier*/);
          /* Go to infinite loop if there is one error */
          if (ret != BLE_STATUS_SUCCESS) {
            while (1) {
              ALLMEMS1_PRINTF("Problem Changing the connection interval\r\n");
            }
          }
        }
        
        /* Signal that we are ready sending back the CRV value*/
        BufferToWrite[0] = PointerByte[0];
        BufferToWrite[1] = PointerByte[1];
        BufferToWrite[2] = PointerByte[2];
        BufferToWrite[3] = PointerByte[3];
        BytesToWrite = 4;
        Term_Update(BufferToWrite,BytesToWrite);
      }
      
      SendBackData=0;      
    } else if(!strncmp("versionBle",(char *)(att_data),10)) {
      uint8_t  hwVersion;
      uint16_t fwVersion;
      /* get the BlueNRG HW and FW versions */
      getBlueNRGVersion(&hwVersion, &fwVersion);
      BytesToWrite =sprintf((char *)BufferToWrite,"%s_%d.%d.%c\r\n",
                            "BlueNRG-MS",
                            fwVersion>>8, 
                            (fwVersion>>4)&0xF,
                            ('a'+(fwVersion&0xF)));
      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
#endif /* DISABLE_FOTA */
    } else if((att_data[0]=='u') & (att_data[1]=='i') & (att_data[2]=='d')) {
      /* Write back the STM32 UID */
      uint8_t *uid = (uint8_t *)STM32_UUID;
      uint32_t MCU_ID = STM32_MCU_ID[0]&0xFFF;
      BytesToWrite =sprintf((char *)BufferToWrite,"%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X_%.3lX\r\n",
                            uid[ 3],uid[ 2],uid[ 1],uid[ 0],
                            uid[ 7],uid[ 6],uid[ 5],uid[ 4],
                            uid[11],uid[ 10],uid[9],uid[8],
                            MCU_ID);
      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
    } else if(!strncmp("setName ",(char *)(att_data),8)) {
      
      //int NameLength= strcspn((const char *)att_data,"\n");
      int NameLength= data_length -1;
      
      if(NameLength > 8)
      {
        for(int i=1;i<8;i++)
          NodeName[i]= atoi(" ");
 
        if((NameLength - 8) > 7)
          NameLength= 7;
        else NameLength= NameLength - 8;
        
        for(int i=1;i<NameLength+1;i++)
          NodeName[i]= att_data[i+7];
        
        MDM_SaveGMD(GMD_NODE_NAME,(void *)&NodeName);
        NecessityToSaveMetaDataManager=1;
        
        BytesToWrite =sprintf((char *)BufferToWrite,"\nThe node nome has been updated\r\n");
        Term_Update(BufferToWrite,BytesToWrite);
        BytesToWrite =sprintf((char *)BufferToWrite,"Disconnecting and riconnecting to see the new node name\r\n");
        Term_Update(BufferToWrite,BytesToWrite);
      }
      else
      {
        BytesToWrite =sprintf((char *)BufferToWrite,"\nInsert the node name\r\n");
        Term_Update(BufferToWrite,BytesToWrite);
        BytesToWrite =sprintf((char *)BufferToWrite,"Use command: setName 'xxxxxxx'\r\n");
        Term_Update(BufferToWrite,BytesToWrite);
      }

      SendBackData=0;
#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
    } else if(!strncmp("setDate",(char *)(att_data),7)) {
      
      int NameLength= data_length -1;
      
      if(NameLength == 19)
      {
        RTC_DateTypeDef StartDate;
        
        StartDate.WeekDay= att_data[9] - 48;
        StartDate.Date=  ((att_data[11] - 48) * 16) + (att_data[12] - 48);
        StartDate.Month= ((att_data[14] - 48) * 16) + (att_data[15] - 48);
        StartDate.Year=  ((att_data[17] - 48) * 16) + (att_data[18] - 48);
        
        if( ((StartDate.WeekDay  > 0x00) && (StartDate.WeekDay  < 0x08)) &&
            ((StartDate.Date     > 0x00) && (StartDate.Date     < 0x32)) &&
            ((StartDate.Month    > 0x00) && (StartDate.Month    < 0x13)) &&
            (StartDate.Year < 0x99) )
        {
          /* Configure RTC Data */
          RTC_DataConfig(StartDate.WeekDay, StartDate.Date, StartDate.Month, StartDate.Year);
        }
        else
        {
          BytesToWrite =sprintf((char *)BufferToWrite,"\nDate format not correct\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
          BytesToWrite =sprintf((char *)BufferToWrite,"\nsetDate wd/dd/mm/yy\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
          BytesToWrite =sprintf((char *)BufferToWrite,"\nwd->(1..7) - dd->(1..31) - mm->(1..12) - yy->(0..99)\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
        }
        
        SendBackData=0;
      }
    } else if(!strncmp("setTime",(char *)(att_data),7)) {
      
      int NameLength= data_length -1;
      
      if(NameLength == 16)
      {
        RTC_TimeTypeDef StartTime;
        
        StartTime.Hours=   ((att_data[8]  - 48) * 16) + (att_data[9]  - 48);
        StartTime.Minutes= ((att_data[11] - 48) * 16) + (att_data[12] - 48);
        StartTime.Seconds= ((att_data[14] - 48) * 16) + (att_data[15] - 48);
        
        if( (StartTime.Hours   < 0x24) &&
            (StartTime.Minutes < 0x60) &&
            (StartTime.Seconds < 0x60) )
        {
          /* Configure RTC Time */
          RTC_TimeConfig(StartTime.Hours, StartTime.Minutes, StartTime.Seconds);
        }
        else
        {
          BytesToWrite =sprintf((char *)BufferToWrite,"\nTime format not correct\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
          BytesToWrite =sprintf((char *)BufferToWrite,"\nsetTime hh:mm:ss\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
          BytesToWrite =sprintf((char *)BufferToWrite,"\nhh->(0..23) - mm->(0..59) - ss->(0..59)\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
        }
        
        SendBackData=0;
      }
    } else if(!strncmp("start",(char *)(att_data),5)) {
        
        SD_Card_Status= 1;
        SD_Card_FeaturesMask= FEATURE_MASK_SENSORFUSION_SHORT | 
                              FEATURE_MASK_TEMP1 |
                              FEATURE_MASK_TEMP2|
                              FEATURE_MASK_PRESS |
                              FEATURE_MASK_HUM |
                              FEATURE_MASK_ACC |
                              FEATURE_MASK_GRYO |
                              FEATURE_MASK_MAG;
        
        SD_Card_StepTime= 0;
        SD_CardLoggingMemsStart();
        
        if(!IsSdMemsRecording)
        {
          SD_Card_Status= 0;
          SD_Card_FeaturesMask= 0x00000000;
        }
        
        SendBackData=0;
      } else if(!strncmp("stop",(char *)(att_data),4)) {
        
        SD_Card_Status= 0;
        SD_Card_FeaturesMask= 0x00000000;
        SD_CardLoggingMemsStop();
        SendBackData=0;
      } else if(!strncmp("AudioStart",(char *)(att_data),10)) {
        
        if(!IsSdMemsRecording)
        {
          if(!IsSdAudioRecording)
          {
            LedOffTargetPlatform();
            openFileAudio();
            
            if(!SD_CardNotPresent)
            {
              InitMics(AUDIO_SAMPLING_FREQUENCY, AUDIO_VOLUME_INPUT, PCM_AUDIO_IN_SAMPLES);
              
              IsSdAudioRecording= 1;

              BytesToWrite =sprintf((char *)BufferToWrite,"Start Data Log Audio Recording\r\n");
              Term_Update(BufferToWrite,BytesToWrite);
            }
            else
            {
              BytesToWrite =sprintf((char *)BufferToWrite,"SD Card not present\r\n");
              Term_Update(BufferToWrite,BytesToWrite);
              SD_CardNotPresent= 0;
              BSP_LED_Init(LED1);  
              LedOffTargetPlatform();
            }
          }
          else
          {
            BytesToWrite =sprintf((char *)BufferToWrite,"Data Log Audio is already started\r\n");
            Term_Update(BufferToWrite,BytesToWrite);
          }
        }
        else
        {
          BytesToWrite =sprintf((char *)BufferToWrite,"Data Log MEMS is already started\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
          BytesToWrite =sprintf((char *)BufferToWrite,"It is need stopped the Data Log MEMS before\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
        }
        
        SendBackData=0;
      } else if(!strncmp("AudioStop",(char *)(att_data),9)) {
        
        if(IsSdAudioRecording)
        {
          BytesToWrite =sprintf((char *)BufferToWrite,"Stop Data Log Audio Recording\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
          
          DeInitMics();
          writeAudio_flag=0;
          closeFileAudio();   
          
          IsSdAudioRecording= 0;
          BSP_LED_Init(LED1);  
          LedOffTargetPlatform();
        }
        else
        {
          BytesToWrite =sprintf((char *)BufferToWrite,"None Data Log Audio\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
        }
      
        SendBackData=0;
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */  
    } else if(!strncmp("TLcalibstart",(char *)(att_data),12)) {
      StartMotionTL_Calibration= 1;
      CalibrationsPerformed= 0;
      BytesToWrite =sprintf((char *)BufferToWrite,"ok\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite =sprintf((char *)BufferToWrite,"Board Position %ld - Write next\r\n", CalibrationsPerformed+1);
      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
    } else if(!strncmp("TLcalibstop",(char *)(att_data),11)) {
      StartMotionTL_Calibration= 0;
      CalibrationsPerformed= 0;
      BytesToWrite =sprintf((char *)BufferToWrite,"ok\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite =sprintf((char *)BufferToWrite,"Calibration Failed\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
    }
    
    if(StartMotionTL_Calibration)
    { 
      if(!strncmp("next",(char *)(att_data),4))
      {
        switch(CalibrationsPerformed)
        {
        case 0:
          MotionTL_manager_calibratePosition(Z_UP);
          CalibrationsPerformed++;
          break;
        case 1:
          MotionTL_manager_calibratePosition(Z_DOWN);
          CalibrationsPerformed++;
          break;
        case 2:
          MotionTL_manager_calibratePosition(Y_UP);
          CalibrationsPerformed++;
          break;
        case 3:
          MotionTL_manager_calibratePosition(X_DOWN);
          CalibrationsPerformed++;
          break;
        case 4:
          MotionTL_manager_calibratePosition(Y_DOWN);
          CalibrationsPerformed++;
          break;
        case 5:
          MotionTL_manager_calibratePosition(X_UP);
          CalibrationsPerformed++;
          break;
        }
        
        if(CalibrationsPerformed == 6)
        {
          MTL_acc_cal_t acc_cal;
          MTL_cal_result_t cal_result = CAL_FAIL;
          
          cal_result = MotionTL_manager_getCalibrationValues(&acc_cal);
          
          if (cal_result == CAL_PASS)
          {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration Done\r\n");
            Term_Update(BufferToWrite,BytesToWrite);
            
            MotionTL_manager_SaveCalValuesInNVM(&acc_cal);
          }
          else
          {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration Failed\r\n");
            Term_Update(BufferToWrite,BytesToWrite);
          }
          
          CalibrationsPerformed= 0;
          StartMotionTL_Calibration= 0;
        }
        else
        {
          BytesToWrite =sprintf((char *)BufferToWrite,"Board Position %ld - Write next\r\n", CalibrationsPerformed+1);
          Term_Update(BufferToWrite,BytesToWrite);
        }
        
        SendBackData=0;
      }
    }

   if(SendBackData) {
    if(att_data[0]=='@') {

      if(att_data[1]=='T') {
        uint8_t loc_att_data[6];
        uint8_t loc_data_length=6;

        loc_att_data[0] = (FEATURE_MASK_TEMP1>>24)&0xFF;
        loc_att_data[1] = (FEATURE_MASK_TEMP1>>16)&0xFF;
        loc_att_data[2] = (FEATURE_MASK_TEMP1>>8 )&0xFF;
        loc_att_data[3] = (FEATURE_MASK_TEMP1    )&0xFF;
        loc_att_data[4] = 255;
        
        switch(att_data[2]) {
          case 'L':
            loc_att_data[5] = 50; /* @5S */
          break;
          case 'M':
            loc_att_data[5] = 10; /* @1S */
          break;
          case 'H':
            loc_att_data[5] = 1; /* @100mS */
          break;
          case 'D':
            loc_att_data[5] = 0; /* Default */
          break;
        }
        SendBackData = ConfigCommandParsing(loc_att_data,loc_data_length);
      } else if(att_data[1]=='A') {
        uint8_t loc_att_data[6];
        uint8_t loc_data_length=6;

        loc_att_data[0] = (FEATURE_MASK_ACC>>24)&0xFF;
        loc_att_data[1] = (FEATURE_MASK_ACC>>16)&0xFF;
        loc_att_data[2] = (FEATURE_MASK_ACC>>8 )&0xFF;
        loc_att_data[3] = (FEATURE_MASK_ACC    )&0xFF;
        loc_att_data[4] = 255;

        switch(att_data[2]) {
          case 'L':
            loc_att_data[5] = 50; /* @5S */
          break;
          case 'M':
            loc_att_data[5] = 10; /* @1S */
          break;
          case 'H':
            loc_att_data[5] = 1; /* @100mS */
          break;
          case 'D':
            loc_att_data[5] = 0; /* Default */
          break;
        }
        SendBackData = ConfigCommandParsing(loc_att_data,loc_data_length);
      } else if(att_data[1]=='M') {
        uint8_t loc_att_data[6];
        uint8_t loc_data_length=6;

        loc_att_data[0] = (FEATURE_MASK_MIC>>24)&0xFF;
        loc_att_data[1] = (FEATURE_MASK_MIC>>16)&0xFF;
        loc_att_data[2] = (FEATURE_MASK_MIC>>8 )&0xFF;
        loc_att_data[3] = (FEATURE_MASK_MIC    )&0xFF;
        loc_att_data[4] = 255;

        switch(att_data[2]) {
          case 'L':
            loc_att_data[5] = 50; /* @5S */
          break;
          case 'M':
            loc_att_data[5] = 10; /* @1S */
          break;
          case 'H':
            loc_att_data[5] = 1; /* @100mS */
          break;
          case 'D':
            loc_att_data[5] = 0; /* Default */
          break;
        }
        SendBackData = ConfigCommandParsing(loc_att_data,loc_data_length);
      }    
    }
  }
  
  return SendBackData;
}

/**
 * @brief  This function makes the parsing of the Configuration Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
static uint32_t ConfigCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  FeatureMask = (att_data[3]) | (att_data[2]<<8) | (att_data[1]<<16) | (att_data[0]<<24);
  uint8_t Command = att_data[4];
  uint8_t Data    = att_data[5];
  uint32_t SendItBack = 1;

  switch (FeatureMask) {
    case FEATURE_MASK_SENSORFUSION_SHORT:
      /* Sensor Fusion */
      switch (Command) {
        case W2ST_COMMAND_CAL_STATUS:
#ifdef ALLMEMS1_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration STATUS Signal For Features=%lx\n\r",FeatureMask);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Calibration STATUS Signal For Features=%lx\n\r",FeatureMask);
          }
#endif /* ALLMEMS1_DEBUG_CONNECTION */
          /* Replay with the calibration status for the feature */
          /* Control the calibration status */
          {
            Config_Notify(FeatureMask,Command,MagnetoCalibrationDone ? 100: 0);
            
          }
        break;
        case W2ST_COMMAND_CAL_RESET:
#ifdef ALLMEMS1_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration RESET Signal For Feature=%lx\n\r",FeatureMask);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Calibration RESET Signal For Feature=%lx\n\r",FeatureMask);
          }
#endif /* ALLMEMS1_DEBUG_CONNECTION */
          /* Reset the calibration */
          ForceReMagnetoCalibration=1;
        break;
        case W2ST_COMMAND_CAL_STOP:
#ifdef ALLMEMS1_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration STOP Signal For Feature=%lx\n\r",FeatureMask);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Calibration STOP Signal For Feature=%lx\n\r",FeatureMask);
          }
#endif /* ALLMEMS1_DEBUG_CONNECTION */
          /* Do nothing in this case */
        break;
        default:
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
            BytesToWrite =sprintf((char *)BufferToWrite, "Calibration UNKNOW Signal For Feature=%lx\n\r",FeatureMask);
            Stderr_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Calibration UNKNOW Signal For Feature=%lx\n\r",FeatureMask);
          }
      }      
    break;
  case FEATURE_MASK_ECOMPASS:
      /* e-compass */
      switch (Command) {
        case W2ST_COMMAND_CAL_STATUS:
#ifdef ALLMEMS1_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration STATUS Signal For Features=%lx\n\r",FeatureMask);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Calibration STATUS Signal For Features=%lx\n\r",FeatureMask);
          }
#endif /* ALLMEMS1_DEBUG_CONNECTION */
          /* Replay with the calibration status for the feature */
          /* Control the calibration status */
          {
            Config_Notify(FeatureMask,Command,MagnetoCalibrationDone ? 100: 0);
          }
        break;
        case W2ST_COMMAND_CAL_RESET:
#ifdef ALLMEMS1_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration RESET Signal For Feature=%lx\n\r",FeatureMask);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Calibration RESET Signal For Feature=%lx\n\r",FeatureMask);
          }
#endif /* ALLMEMS1_DEBUG_CONNECTION */
          /* Reset the calibration */
          ForceReMagnetoCalibration=2;
        break;
        case W2ST_COMMAND_CAL_STOP:
#ifdef ALLMEMS1_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration STOP Signal For Feature=%lx\n\r",FeatureMask);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Calibration STOP Signal For Feature=%lx\n\r",FeatureMask);
          }
#endif /* ALLMEMS1_DEBUG_CONNECTION */
          /* Do nothing in this case */
        break;
        default:
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
            BytesToWrite =sprintf((char *)BufferToWrite, "Calibration UNKNOW Signal For Feature=%lx\r\n",FeatureMask);
            Stderr_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Calibration UNKNOW Signal For Feature=%lx\r\n",FeatureMask);
          }
      }  
    break;
  case FEATURE_MASK_ACC_EVENTS:
    /* Acc events */
#ifdef ALLMEMS1_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%c D=%x\r\n",FeatureMask,Command,Data);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      ALLMEMS1_PRINTF("Conf Sig F=%lx C=%c D=%x\r\n",FeatureMask,Command,Data);
    }
#endif /* ALLMEMS1_DEBUG_CONNECTION */      
    switch(Command) {
      case 'm':
        /* Multiple Events */
        switch(Data) {
          case 1:
            EnableHWMultipleEvents();
            ResetHWPedometer();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWMultipleEvents();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
        break;
      case 'f':
        /* FreeFall */
        switch(Data) {
          case 1:
            EnableHWFreeFall();
             Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWFreeFall();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
      break;
      case 'd':
        /* Double Tap */
        switch(Data) {
          case 1:
            EnableHWDoubleTap();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWDoubleTap();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
      break;
      case 's':
        /* Single Tap */
        switch(Data) {
          case 1:
            EnableHWSingleTap();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWSingleTap();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
      break;
      case 'p':
        /* Pedometer */
        switch(Data) {
          case 1:
            EnableHWPedometer();
            ResetHWPedometer();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWPedometer();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
       break;
      case 'w':
        /* Wake UP */
        switch(Data) {
          case 1:
            EnableHWWakeUp();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWWakeUp();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
       break;
       case 't':
         /* Tilt */
        switch(Data) {
          case 1:
            EnableHWTilt();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWTilt();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
      break;
      case 'o' :
        /* Tilt */
        switch(Data) {
        case 1:
          EnableHWOrientation6D();
          Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
          break;
        case 0:
          DisableHWOrientation6D();
          Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
          break;
        }
      break;
    }
    break;
    /* Environmental features */
    case FEATURE_MASK_TEMP1:
    case FEATURE_MASK_TEMP2:
    case FEATURE_MASK_PRESS:
    case FEATURE_MASK_HUM:
      switch(Command) {
        case 255:
          /* Change the Sending interval */
          if(Data!=0) {
            /* Multiple of 100mS */
            __HAL_TIM_SET_AUTORELOAD(&TimEnvHandle,(((Data*TIM_CLOCK_ENV) / 10U) - 1));
            __HAL_TIM_SET_COUNTER(&TimEnvHandle,0);
            TimEnvHandle.Instance->EGR = TIM_EGR_UG;
          } else {
            /* Default Values */
            __HAL_TIM_SET_AUTORELOAD(&TimEnvHandle,((TIM_CLOCK_ENV  / ALGO_FREQ_ENV) - 1));
            __HAL_TIM_SET_COUNTER(&TimEnvHandle,0);
          }
#ifdef ALLMEMS1_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
          }
#endif /* ALLMEMS1_DEBUG_CONNECTION */
          SendItBack = 0;
        break;
      }
    break;
    /* Inertial features */
    case FEATURE_MASK_ACC:
    case FEATURE_MASK_GRYO:
    case FEATURE_MASK_MAG:
      switch(Command) {
        case 255:
          /* Change the Sending interval */
          if(Data!=0) {
            /* Multiple of 100mS */
            uhCCR4_Val  = 1000*Data;
          } else {
            /* Default Value */
            uhCCR4_Val  = DEFAULT_uhCCR4_Val;
          }
#ifdef ALLMEMS1_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
          }
#endif /* ALLMEMS1_DEBUG_CONNECTION */
          SendItBack = 0;
        break;
      }
    break;
    /* Environmental features */
    case FEATURE_MASK_MIC:
      switch(Command) {
        case 255:
          /* Change the Sending interval */
          if(Data!=0) {
            /* Multiple of 100mS */
            __HAL_TIM_SET_AUTORELOAD(&TimAudioDataHandle,(((Data*TIM_CLOCK_AUDIO_LEVEL) / 10U) - 1));
            __HAL_TIM_SET_COUNTER(&TimAudioDataHandle,0);
            TimAudioDataHandle.Instance->EGR = TIM_EGR_UG;
          } else {
            /* Default Values */
            __HAL_TIM_SET_AUTORELOAD(&TimAudioDataHandle,((TIM_CLOCK_AUDIO_LEVEL / ALGO_FREQ_AUDIO_LEVEL) - 1));
            __HAL_TIM_SET_COUNTER(&TimAudioDataHandle,0);
          }
#ifdef ALLMEMS1_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
          }
#endif /* ALLMEMS1_DEBUG_CONNECTION */
          SendItBack = 0;
        break;
      }
    break;
  }
  return SendItBack;
}

/**
 * @brief  Force one UUID rescan
 * @param  None
 * @retval None
 */
static void Force_UUID_Rescan(void)
{
  tBleStatus ret;
  
  uint8_t buff[4];

  /* Delete all the Handles from 0x0001 to 0xFFFF */
  STORE_LE_16(buff  ,0x0001);
  STORE_LE_16(buff+2,0xFFFF);

  ret = aci_gatt_update_char_value(0x0001,0x0002,0,4,buff);

  if (ret == BLE_STATUS_SUCCESS){
    ALLMEMS1_PRINTF("UUID Rescan Forced\r\n\r\n");
  } else {
    ALLMEMS1_PRINTF("Problem forcing UUID Rescan\r\n\r\n");
  }
}

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
/**
 * @brief  This function makes the parsing of the Commands for SD Card Logging Support
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
static uint32_t SD_CardLogging_CommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t SendItBack = 1;

  SD_Card_Status      = att_data[0];
  
  if(SD_Card_Status)
  {
    SD_Card_FeaturesMask= (att_data[4] << 24) | (att_data[3] << 16) | (att_data[2] << 8 ) | (att_data[1]);
    SD_Card_StepTime    = (att_data[8] << 24) | (att_data[7] << 16) | (att_data[6] << 8 ) | (att_data[5]);
    
    SD_CardLogging=1;
    
//    __HAL_RTC_WRITEPROTECTION_DISABLE(&RtcHandle);
//    HAL_RTC_WaitForSynchro(&RtcHandle);
//    __HAL_RTC_WRITEPROTECTION_ENABLE(&RtcHandle);
//    RTC_GetCurrentDateTime();
    
//    SD_CardLoggingMemsStart();
//    
//    if(!IsSdMemsRecording)
//    {
//      SD_Card_Status= 0;
//      SD_CardLoggingStatus_Notify(2, SD_Card_FeaturesMask, SD_Card_StepTime);
//    }
//    else
//    {
//      SD_CardLoggingMemsData();
//    }
  }
  else
  {
    SD_CardLoggingMemsStop();
    SD_CardLoggingStatus_Notify(SD_Card_Status, SD_Card_FeaturesMask, SD_Card_StepTime);
  }
  
  return SendItBack;
}
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void *pckt Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT) {
    return;
  }
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data; 
          Read_Request_CB(pr->attr_handle);                    
        }
        break;
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
          Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
        }
        break;
      }
    }
    break;
  }
}

