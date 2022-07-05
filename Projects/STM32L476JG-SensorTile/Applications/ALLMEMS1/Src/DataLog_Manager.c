/**
  ******************************************************************************
  * @file    DataLog_Manager.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file includes Source location interface functions
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
#include "TargetFeatures.h"
#include "sensor_service.h"
#include "DataLog_Manager.h"
#include "datalog_application.h"

#include <stdio.h>

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING

/* Exported Variables -------------------------------------------------------------*/
volatile int  index_buff=0;
volatile uint8_t writeAudio_flag=0;

uint8_t SD_CardLogging_StepHours;
uint8_t SD_CardLogging_StepMinutes;
uint8_t SD_CardLogging_StepSeconds;

uint16_t Audio_OUT_Buff[AUDIO_BUFF_SIZE];


/* Imported Variables -------------------------------------------------------------*/
extern volatile uint8_t SD_Log_Enabled;
extern volatile uint8_t SD_LogAudio_Enabled;

extern volatile uint32_t SD_CardLogging;

/* Code for MotionFX integration - Start Section */
extern BSP_MOTION_SENSOR_Axes_t MAG_Offset;
/* Code for MotionFX integration - End Section */

/* Feature mask that identify the data mens selected for recording*/
extern uint32_t SD_Card_FeaturesMask;
/* Time range for data recording in second */
extern uint32_t SD_Card_StepTime;

/* Enable ShutDown Mode as default with SD card logging features*/
extern uint8_t ShutDownAllowed;

extern uint16_t sdcard_file_counter;

extern uint8_t IsSdMemsRecording;
extern uint8_t IsSdAudioRecording;
extern volatile uint8_t SD_CardNotPresent;

extern uint32_t  DataLogStatus[];

/* RTC handler declared in "main.c" file */
extern RTC_HandleTypeDef RtcHandle;

extern TIM_HandleTypeDef    TimSdRecordingHandle;

extern uint16_t PCM_Buffer[];

/* Private variables -------------------------------------------------------------*/
static char myBuffer[ARRAYSIZE];

/* Private function prototypes ---------------------------------------------------*/
static BSP_MOTION_SENSOR_Axes_t Accelero_Sensor_Handler_Light(void);
static BSP_MOTION_SENSOR_Axes_t Gyro_Sensor_Handler_Light(void);
static BSP_MOTION_SENSOR_Axes_t Magneto_Sensor_Handler_Light(void);
static float Temperature1_Sensor_Handler_Light(void);
static float Temperature2_Sensor_Handler_Light(void);
static float Pressure_Sensor_Handler_Light(void);
static float Humidity_Sensor_Handler_Light(void);


/**
  * @brief  Management of the audio data logging
  * @param  None
  * @retval None
  */
void AudioProcess_SD_Recording(void)
{
  uint16_t index = 0;
  static uint16_t OUT_Buff_lvl = 0;
  
  for (index = 0; index < AUDIO_SAMPLING_FREQUENCY/1000 ; index++)
  {
    Audio_OUT_Buff[OUT_Buff_lvl] = PCM_Buffer[index*AUDIO_CHANNELS];
    OUT_Buff_lvl = (OUT_Buff_lvl + 1)%AUDIO_BUFF_SIZE;
  }
  
  //first half
  if(OUT_Buff_lvl == AUDIO_BUFF_SIZE/2)
  {
    index_buff=0;
    writeAudio_flag=1; 
  }    
  //second half
  else if (OUT_Buff_lvl == 0)
  {
    index_buff= AUDIO_BUFF_SIZE;
    writeAudio_flag=1; 
  }    
}

/**
  * @brief  Management of the audio file opening
  * @param  None
  * @retval None
  */
void openFileAudio(void)
{  
  while( (SD_LogAudio_Enabled != 1) &&
         (SD_CardNotPresent != 1) )
  {
    if(DATALOG_SD_LogAudio_Enable())
    {
      SD_LogAudio_Enabled=1;
    }
    else
    {
      DATALOG_SD_LogAudio_Disable();
      //DATALOG_SD_Log_Disable();
      DATALOG_SD_DeInit();
      DATALOG_SD_Init();
    }
    HAL_Delay(100);
  }
}

/**
  * @brief  Management of the audio file closing
  * @param  None
  * @retval None
  */
void closeFileAudio(void)
{
  DATALOG_SD_LogAudio_Disable();
  SD_LogAudio_Enabled=0;
}

/**
  * @brief  Management of the data logging
  * @param  None
  * @retval None
  */
void SD_CardLoggingMemsData(void)
{
  BSP_MOTION_SENSOR_Axes_t Acceleration;
  BSP_MOTION_SENSOR_Axes_t AngularVelocity;
  BSP_MOTION_SENSOR_Axes_t Magnetometer;
  
  MFX_input_t data_in;
  MFX_input_t *pdata_in = &data_in;
  MFX_output_t data_out;
  MFX_output_t *pdata_out = &data_out;
  
  float Temperature_1= 0;
  float Temperature_2= 0;
  float Pressure     = 0;
  float Humidity     = 0;
  
  float Quat[4]= {0.0};
 
  Acceleration.x= 0;
  Acceleration.y= 0;
  Acceleration.z= 0;
  
  AngularVelocity.x= 0;
  AngularVelocity.y= 0;
  AngularVelocity.z= 0;
  
  Magnetometer.x= 0;
  Magnetometer.y= 0;
  Magnetometer.z= 0;
  
  if(SD_Card_FeaturesMask & FEATURE_MASK_SENSORFUSION_SHORT)
  {
    Acceleration= Accelero_Sensor_Handler_Light();
    Magnetometer= Magneto_Sensor_Handler_Light();
    AngularVelocity= Gyro_Sensor_Handler_Light();
    
    /* Convert acceleration from [mg] to [g] */
    data_in.acc[0] = (float)Acceleration.x * FROM_MG_TO_G;
    data_in.acc[1] = (float)Acceleration.y * FROM_MG_TO_G;
    data_in.acc[2] = (float)Acceleration.z * FROM_MG_TO_G;

    /* Convert magnetic field intensity from [mGauss] to [uT / 50] */
    data_in.mag[0] = (float)(Magnetometer.x - MAG_Offset.x) * FROM_MGAUSS_TO_UT50;
    data_in.mag[1] = (float)(Magnetometer.y - MAG_Offset.y) * FROM_MGAUSS_TO_UT50;
    data_in.mag[2] = (float)(Magnetometer.z - MAG_Offset.z) * FROM_MGAUSS_TO_UT50;

    /* Convert angular velocity from [mdps] to [dps] */
    data_in.gyro[0] = (float)AngularVelocity.x * FROM_MDPS_TO_DPS;
    data_in.gyro[1] = (float)AngularVelocity.y * FROM_MDPS_TO_DPS;
    data_in.gyro[2] = (float)AngularVelocity.z * FROM_MDPS_TO_DPS;
    
    MotionFX_manager_run(pdata_in, pdata_out, MOTION_FX_ENGINE_DELTATIME);
    
    Quat[0]= pdata_out->quaternion[0];
    Quat[1]= pdata_out->quaternion[1];
    Quat[2]= pdata_out->quaternion[2];
    Quat[3]= pdata_out->quaternion[3];
    
    AngularVelocity.x= 0;
    AngularVelocity.y= 0;
    AngularVelocity.z= 0;
    
    Magnetometer.x= 0;
    Magnetometer.y= 0;
    Magnetometer.z= 0;
  }

  if(SD_Card_FeaturesMask & FEATURE_MASK_ACC)
    Acceleration= Accelero_Sensor_Handler_Light();
  
  if(SD_Card_FeaturesMask & FEATURE_MASK_GRYO)
    AngularVelocity= Gyro_Sensor_Handler_Light();
  
  if(SD_Card_FeaturesMask & FEATURE_MASK_MAG)
    Magnetometer= Magneto_Sensor_Handler_Light();
  
  if(SD_Card_FeaturesMask & FEATURE_MASK_PRESS)
    Pressure= Pressure_Sensor_Handler_Light();
  
  if(SD_Card_FeaturesMask & FEATURE_MASK_TEMP1)
    Temperature_1= Temperature1_Sensor_Handler_Light();
  
  if(SD_Card_FeaturesMask & FEATURE_MASK_TEMP2)
    Temperature_2= Temperature2_Sensor_Handler_Light();
 
  if(SD_Card_FeaturesMask & FEATURE_MASK_HUM)
    Humidity= Humidity_Sensor_Handler_Light();
  
  openFile(OPEN_FILE_FOR_APP);
  saveData(myBuffer, Quat, Acceleration, AngularVelocity, Magnetometer, Pressure, Temperature_1, Temperature_2, Humidity, FREQUENCY);
  closeFile();
}

/**
  * @brief  Management of the file opening
  * @param  None
  * @retval None
  */
void openFile(uint8_t AccessControl)
{  
  while( (SD_Log_Enabled != 1) &&
         (SD_CardNotPresent != 1) )
  {
    if(DATALOG_SD_Log_Enable(AccessControl))
    {
      SD_Log_Enabled=1;
    }
    else
    {
      DATALOG_SD_Log_Disable();
      DATALOG_SD_DeInit();
      DATALOG_SD_Init();
    }
    HAL_Delay(100);
  }
}

/**
  * @brief  Management of the file closing
  * @param  None
  * @retval None
  */
void closeFile(void)
{
  DATALOG_SD_Log_Disable();
  SD_Log_Enabled=0;
}

/**
 * @brief  Management of the start of the SD Card data logging
 * @param  None
 * @retval None
 */
void SD_CardLoggingMemsStart(void)
{
  if(!IsSdAudioRecording)
  {
    if(!IsSdMemsRecording)
    {
      LedOffTargetPlatform();
      openFile(CREATE_FILE_FOR_WRITE);
     
      if(!SD_CardNotPresent)
      {
        IsSdMemsRecording= 1;
        closeFile();
        
        DataLogStatus[0]= (uint32_t)0x12345678;
        DataLogStatus[1]= (uint32_t)0x00000001;
        DataLogStatus[2]= SD_Card_FeaturesMask;
        DataLogStatus[3]= SD_Card_StepTime;
        DataLogStatus[4]= (uint32_t)sdcard_file_counter;
        MDM_SaveGMD(GMD_DATA_LOG_STATUS,(void *)&DataLogStatus);
        NecessityToSaveMetaDataManager=1;
        
        if(SD_Card_StepTime > 0)
        {
          if(SD_Card_StepTime < (RANGE_TIME_WITHOUT_CONNECTED/1000))
          {
            ShutDownAllowed= 0;
          }
        }
        else
        {
          /* Start the TIM Base generation in interrupt mode */
          if(HAL_TIM_Base_Start_IT(&TimSdRecordingHandle) != HAL_OK){
            /* Starting Error */
            Error_Handler();
          }
          
          ShutDownAllowed= 0;
        }
        
        BytesToWrite =sprintf((char *)BufferToWrite,"Start Data Log MEMS Recording\r\n");
        Term_Update(BufferToWrite,BytesToWrite);
      }
      else
      {
        BytesToWrite =sprintf((char *)BufferToWrite,"SD Card not present\r\n");
        Term_Update(BufferToWrite,BytesToWrite);
        SD_CardNotPresent= 0;
        BSP_LED_Init(LED1);  
        BSP_LED_Off(LED1);
      }
    }
    else
    {
      BytesToWrite =sprintf((char *)BufferToWrite,"Data Log MEMS is already started\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
    }
  }
  else
  {
    BytesToWrite =sprintf((char *)BufferToWrite,"Data Log Audio is already started\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
    BytesToWrite =sprintf((char *)BufferToWrite,"It is need stopped the Data Log Audio before\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  }
}

/**
 * @brief  Management of the stop of the SD Card data logging
 * @param  None
 * @retval None
 */
void SD_CardLoggingMemsStop(void)
{
  if(IsSdMemsRecording)
  {
    BytesToWrite =sprintf((char *)BufferToWrite,"Stop Data Log MEMS Recording\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
    
    /* Deactivate alarm if it was activated */
    HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_A);
    
    /* Stop Timer For Sd Card Recording if it was started */
    if(HAL_TIM_Base_Stop_IT(&TimSdRecordingHandle) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
    
    ShutDownAllowed= 1;
    SD_CardLogging=0;
    
    IsSdMemsRecording= 0;
//    BSP_LED_Init(LED1);  
//    LedOffTargetPlatform();
    
    DataLogStatus[0]= (uint32_t)0x12345678;
    DataLogStatus[1]= (uint32_t)0x00000000;
    DataLogStatus[2]= SD_Card_FeaturesMask;
    DataLogStatus[3]= SD_Card_StepTime;
    DataLogStatus[4]= (uint32_t)sdcard_file_counter;
    MDM_SaveGMD(GMD_DATA_LOG_STATUS,(void *)&DataLogStatus);
    NecessityToSaveMetaDataManager=1;
  }
  else
  {
    BytesToWrite =sprintf((char *)BufferToWrite,"None Data Log MEMS\r\n");
    Term_Update(BufferToWrite,BytesToWrite);          
  }
}
    
/**
 * @brief  Handles the accelerometer axes data, fast
 * @param  None
 * @retval BSP_MOTION_SENSOR_Axes_t acceleration
 */
static BSP_MOTION_SENSOR_Axes_t Accelero_Sensor_Handler_Light(void)
{
  BSP_MOTION_SENSOR_Axes_t acceleration;
  
  acceleration.x = 0;
  acceleration.y = 0;
  acceleration.z = 0;

  if(BSP_MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE,MOTION_ACCELERO,&acceleration) != BSP_ERROR_NONE)
  {
    acceleration.x = 0;
    acceleration.y = 0;
    acceleration.z = 0;
  }
  
  return acceleration;
}

/**
* @brief  Handles the gyroscope axes data getting/sending
* @param  None
* @retval BSP_MOTION_SENSOR_Axes_t angular_velocity
*/
static BSP_MOTION_SENSOR_Axes_t Gyro_Sensor_Handler_Light(void)
{
  BSP_MOTION_SENSOR_Axes_t angular_velocity;
  
  angular_velocity.x = 0;
  angular_velocity.y = 0;
  angular_velocity.z = 0;
  
  if(BSP_MOTION_SENSOR_GetAxes(GYRO_INSTANCE,MOTION_GYRO, &angular_velocity) != BSP_ERROR_NONE)
  {
    angular_velocity.x = 0;
    angular_velocity.y = 0;
    angular_velocity.z = 0;
  }
  
  return angular_velocity;
}

/**
* @brief  Handles the magneto axes data getting/sending
* @param  None
* @retval BSP_MOTION_SENSOR_Axes_t magnetic_field
*/
static BSP_MOTION_SENSOR_Axes_t Magneto_Sensor_Handler_Light(void)
{
  BSP_MOTION_SENSOR_Axes_t magnetic_field;
  
  magnetic_field.x = 0;
  magnetic_field.y = 0;
  magnetic_field.z = 0;
  
  if(BSP_MOTION_SENSOR_GetAxes(MAGNETO_INSTANCE,MOTION_MAGNETO, &magnetic_field) == BSP_ERROR_NONE)
  {
    /* Apply Magneto calibration */
    magnetic_field.x = magnetic_field.x - MAG_Offset.x;
    magnetic_field.y = magnetic_field.y - MAG_Offset.y;
    magnetic_field.z = magnetic_field.z - MAG_Offset.z;
  }
  else
  {
    magnetic_field.x = 0;
    magnetic_field.y = 0;
    magnetic_field.z = 0;
  }
  
  return magnetic_field; 
}

/**
* @brief  Handles the temperature data getting/sending
* @param  None
* @retval float temperature
*/
static float Temperature1_Sensor_Handler_Light(void)
{
  float temperature = 0.0;
  
  if(BSP_ENV_SENSOR_GetValue(TEMPERATURE_INSTANCE_1,ENV_TEMPERATURE,(float *)&temperature) != BSP_ERROR_NONE)
  {
    temperature = 0.0f;
  }
  
  return temperature;
}

/**
* @brief  Handles the temperature data getting/sending
* @param  None
* @retval float temperature
*/
static float Temperature2_Sensor_Handler_Light(void)
{
  float temperature = 0.0;
  
  if(BSP_ENV_SENSOR_GetValue(TEMPERATURE_INSTANCE_2,ENV_TEMPERATURE,(float *)&temperature) != BSP_ERROR_NONE)
  {
    temperature = 0.0f;
  }
  
  return temperature;
}

/**
* @brief  Handles the pressure sensor data getting/sending
* @param  None
* @retval float pressure
*/
static float Pressure_Sensor_Handler_Light(void)
{
  float pressure= 0.0f;
  
  if(BSP_ENV_SENSOR_GetValue(PRESSURE_INSTANCE,ENV_PRESSURE,(float *)&pressure) != BSP_ERROR_NONE)
  {
    pressure = 0.0f;
  }
  
  return pressure;
}

/**
* @brief  Handles the humidity data getting/sending
* @param  None
* @retval float humidity
*/
static float Humidity_Sensor_Handler_Light(void)
{
  float humidity= 0.0f;
  
  if(BSP_ENV_SENSOR_GetValue(HUMIDITY_INSTANCE,ENV_HUMIDITY,(float *)&humidity) != BSP_ERROR_NONE)
  {
    humidity = 0.0f;
  }
  
  return humidity;
}

#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

