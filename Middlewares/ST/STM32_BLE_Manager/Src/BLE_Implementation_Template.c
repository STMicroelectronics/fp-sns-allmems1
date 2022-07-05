/**
  ******************************************************************************
  * @file    BLE_Implementation.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.1.0
  * @date    23-Dec-2021
  * @brief   BLE Implementation template file.
  *          This file should be copied to the application folder and renamed
  *          to BLE_Implementation.c.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "BLE_Manager.h"

/* Exported Variables --------------------------------------------------------*/
int32_t  NeedToClearSecureDB=0;
int32_t needToResetBLE=0;


/* Static Variables ----------------------------------------------------------*/
static uint16_t CurrentConnectionHandle=0;
    
static uint16_t CustomCommandPageLevel=0;
      
/* Private functions ---------------------------------------------------------*/
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length);
static void ReadRequestEnvFunction(void);
static void DisconnectionCompletedFunction(void);
static void ConnectionCompletedFunction(uint16_t ConnectionHandle);

/* For Aci Gatt Tx Pool Available Event */
static void AciGattTxPoolAvailableEventFunction(void);

/**********************************************************************************************
 * Callback functions prototypes to manage the extended configuration characteristic commands *
 **********************************************************************************************/
static void ExtConfigCustomCommandCallback(BLE_CustomCommadResult_t *CustomCommand);
static void ExtConfigRebootOnDFUModeCommandCallback(void);
static void ExtConfigPowerOffCommandCallback(void);
static void ExtConfigSetNameCommandCallback(uint8_t *NewName);
static void ExtConfigReadCustomCommandsCallback(JSON_Array *JSON_SensorArray);
static void ExtConfigReadSensorConfigCommandCallback(JSON_Array *JSON_SensorArray);
static void ExtConfigSetTimeCommandCallback(uint8_t *NewTime);
static void ExtConfigSetDateCommandCallback(uint8_t *NewDate);
static void ExtConfigSetWiFiCommandCallback(BLE_WiFi_CredAcc_t NewWiFiCred);
static void ExtConfigChangePinCommandCallback(uint32_t NewPin);
static void ExtConfigReadCertCommandCallback(uint8_t *Certificate);
static void ExtConfigClearDBCommandCallback();
static void ExtConfigSetCertCommandCallback(uint8_t *Certificate);
static void ExtExtConfigUidCommandCallback(uint8_t **UID);
static void ExtConfigInfoCommandCallback(uint8_t *Answer);
static void ExtConfigHelpCommandCallback(uint8_t *Answer);
static void ExtConfigPowerStatusCommandCallback(uint8_t *Answer);
static void ExtConfigVersionFwCommandCallback(uint8_t *Answer);
static void ExtConfigSetSensorConfigCommandCallback(uint8_t *configuration);



/** @brief Initialize the BlueNRG stack and services
* @param  None
* @retval None
*/
void BluetoothInit(void)
{
  /* BlueNRG stack setting */
  BlueNRG_StackValue.ConfigValueOffsets                   = CONFIG_DATA_PUBADDR_OFFSET;
  BlueNRG_StackValue.ConfigValuelength                    = CONFIG_DATA_PUBADDR_LEN;
  BlueNRG_StackValue.GAP_Roles                            = GAP_PERIPHERAL_ROLE;
  BlueNRG_StackValue.IO_capabilities                      = IO_CAP_DISPLAY_ONLY;
  BlueNRG_StackValue.AuthenticationRequirements           = BONDING;
  BlueNRG_StackValue.MITM_ProtectionRequirements          = MITM_PROTECTION_REQUIRED;
  BlueNRG_StackValue.SecureConnectionSupportOptionCode    = SC_IS_SUPPORTED;
  BlueNRG_StackValue.SecureConnectionKeypressNotification = KEYPRESS_IS_NOT_SUPPORTED;
  
  /* To set the TX power level of the bluetooth device ( -2,1 dBm )*/
  BlueNRG_StackValue.EnableHighPowerMode= 1; /*  High Power */
  
  /* Values: 0x00 ... 0x31 - The value depends on the device */
  BlueNRG_StackValue.PowerAmplifierOutputLevel =4;
  
  /* BlueNRG services setting */
  BlueNRG_StackValue.EnableConfig    = 1;
  BlueNRG_StackValue.EnableConsole   = 1;
  BlueNRG_StackValue.EnableExtConfig = 1;
  
  /* For Enabling the Secure Connection */
  BlueNRG_StackValue.EnableSecureConnection=1;
  /* Default Secure PIN */
  BlueNRG_StackValue.SecurePIN=123456;
  
  /* For creating a Random Secure PIN */
#ifdef BLE_MANAGER_PRINTF
  BlueNRG_StackValue.EnableRandomSecurePIN = 1;
#else /* BLE_MANAGER_PRINTF */
  BlueNRG_StackValue.EnableRandomSecurePIN = 0;
#endif /* BLE_MANAGER_PRINTF */
  
  BlueNRG_StackValue.AdvertisingFilter    = NO_WHITE_LIST_USE;
  
  if(BlueNRG_StackValue.EnableSecureConnection) {
    /* Using the Secure Connection, the Rescan should be done by BLE chip */    
    BlueNRG_StackValue.ForceRescan =0;
  } else {
    BlueNRG_StackValue.ForceRescan =1;
  }
  
  InitBleManager();
}

/**
 * @brief  Custom Service Initialization.
 * @param  None
 * @retval None
 */
void BLE_InitCustomService(void) {
  /* Define Custom Function for Debug Console Command parsing */
  CustomDebugConsoleParsingCallback = &DebugConsoleCommandParsing;
  
  /* Define Custom Function for Connection Completed */
  CustomConnectionCompleted = &ConnectionCompletedFunction;
  
  /* Define Custom Function for Disconnection Completed */
  CustomDisconnectionCompleted = &DisconnectionCompletedFunction;
  
  /***********************************************************************************
   * Callback functions to manage the extended configuration characteristic commands *
   ***********************************************************************************/
  CustomExtConfigCustomCommandCallback = &ExtConfigCustomCommandCallback;
  CustomExtConfigUidCommandCallback = ExtExtConfigUidCommandCallback;
  CustomExtConfigRebootOnDFUModeCommandCallback = &ExtConfigRebootOnDFUModeCommandCallback;
  CustomExtConfigPowerOffCommandCallback = &ExtConfigPowerOffCommandCallback;
  CustomExtConfigSetNameCommandCallback = &ExtConfigSetNameCommandCallback;
  CustomExtConfigReadCustomCommandsCallback = &ExtConfigReadCustomCommandsCallback;
  CustomExtConfigSetTimeCommandCallback = &ExtConfigSetTimeCommandCallback;
  CustomExtConfigSetDateCommandCallback = &ExtConfigSetDateCommandCallback;
  CustomExtConfigSetWiFiCommandCallback = &ExtConfigSetWiFiCommandCallback;
  CustomExtConfigChangePinCommandCallback = &ExtConfigChangePinCommandCallback;
  CustomExtConfigReadCertCommandCallback = &ExtConfigReadCertCommandCallback;
  CustomExtConfigClearDBCommandCallback = &ExtConfigClearDBCommandCallback;
  CustomExtConfigSetCertCommandCallback = &ExtConfigSetCertCommandCallback;
  CustomExtConfigInfoCommandCallback = &ExtConfigInfoCommandCallback;
  CustomExtConfigHelpCommandCallback = &ExtConfigHelpCommandCallback;
  CustomExtConfigPowerStatusCommandCallback = &ExtConfigPowerStatusCommandCallback;
  CustomExtConfigVersionFwCommandCallback = &ExtConfigVersionFwCommandCallback;
  CustomExtConfigReadSensorsConfigCommandsCallback = &ExtConfigReadSensorConfigCommandCallback;
  CustomExtConfigSetSensorsConfigCommandsCallback =  &ExtConfigSetSensorConfigCommandCallback;
  
  /**
  * For each features, user can assign here the pointer at the function for the read request data.
  * For example for the environmental features:
  * 
  * CustomReadRequestEnv = &ReadRequestEnvFunction;
  * 
  * User can define and insert in the BLE_Implementation.c source code the functions for the read request data
  * ReadRequestEnvFunction function is already defined.
  *
  */
  
  /**
  * User can added here the custom service initialization for the selected BLE features.
  * For example for the environmental features:
  * 
  * //BLE_InitEnvService(PressEnable,HumEnable,NumTempEnabled)
  * BleManagerAddChar(BleCharPointer= BLE_InitEnvService(1, 1, 1));
  */
}

/**
 * @brief  Set Custom Advertize Data.
 * @param  uint8_t *manuf_data: Advertize Data
 * @retval None
 */
void BLE_SetCustomAdvertizeData(uint8_t *manuf_data)
{
  /**
  * User can add here the custom advertize data setting  for the selected BLE features.
  */
  
#ifndef BLE_MANAGER_SDKV2
  /**
  * User can add here the custom advertize data setting  for the selected BLE features.
  * For example for the environmental features:
  * 
  *		//Custom advertize data setting for the environmental features
  *		BLE_SetEnvAdvertizeData(manuf_data);
  */
#else /* BLE_MANAGER_SDKV2 */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD1]=0x03; /* TMP Code BLE_PROPOSAL */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD2]=0x03; /* 3->30% battery Level */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD3]=0x33; /* Dummy */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD4]=0x44; /* Dummy */
#endif /* BLE_MANAGER_SDKV2 */
}


/**
 * @brief  Aci Gatt Tx Pool Available Event Function.
 * @param  None
 * @retval None
 */
static void AciGattTxPoolAvailableEventFunction(void)
{
  BLE_MANAGER_PRINTF("Call to AciGattTxPoolAvailableEventFunction\r\n");
}

/**
 * @brief  This function makes the parsing of the Debug Console Commands
 * @param  uint8_t *att_data attribute data
 * @param  uint8_t data_length length of the data
 * @retval uint32_t SendBackData true/false
 */
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t SendBackData = 1;

    /* Help Command */
    if(!strncmp("help",(char *)(att_data),4)) {
      /* Print Legend */
      SendBackData=0;

      BytesToWrite =sprintf((char *)BufferToWrite,
         "help\n");
      Term_Update(BufferToWrite,BytesToWrite);
    }
	
	/* Add here the parsing for others commands in the debug console  */
	
  return SendBackData;
}

/**
 * @brief  This function is called when there is a Bluetooth Read request.
 * @param  None 
 * @retval None
 */
static void ReadRequestEnvFunction(void)
{
  /* Read Request for Pressure,Humidity, and Temperatures*/
  int32_t PressToSend;
  uint16_t HumToSend;
  int16_t TempToSend;

  /* Read all the Environmental Sensors */
  ReadEnvironmentalData(&PressToSend,&HumToSend, &TempToSend);
  
  /* Send the Data with BLE */
  BLE_EnvironmentalUpdate(PressToSend,HumToSend,TempToSend, 0);
  
  BLE_MANAGER_PRINTF("Read for Env\r\n");
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void DisconnectionCompletedFunction(void)
{
  BLE_MANAGER_PRINTF("Call to DisconnectionCompletedFunction\r\n");
  CurrentConnectionHandle =0;
  BLE_MANAGER_DELAY(100);
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  None 
 * @retval None
 */
static void ConnectionCompletedFunction(uint16_t ConnectionHandle)
{
  BLE_MANAGER_PRINTF("Call to ConnectionCompletedFunction\r\n");
  CurrentConnectionHandle=ConnectionHandle;
  BLE_MANAGER_DELAY(100);
}

/***********************************************************************************
 * Callback functions to manage the extended configuration characteristic commands *
 ***********************************************************************************/

/**
 * @brief  Callback Function for managing the custom command
 * @param  BLE_CustomCommadResult_t *CustomCommand:
 * @param                            uint8_t *CommandName: Nome of the command
 * @param                            CustomCommand->CommandType: Type of the command
 * @param                            int32_t IntValue:    Integer or boolean parameter
 * @param                            uint8_t *StringValue: String parameter
 * @retval None
 */
static void  ExtConfigCustomCommandCallback(BLE_CustomCommadResult_t *CustomCommand)
{
  BLE_MANAGER_PRINTF("Received Custom Command:\r\n");
  BLE_MANAGER_PRINTF("\tCommand Name: <%s>\r\n", CustomCommand->CommandName);
  BLE_MANAGER_PRINTF("\tCommand Type: <%d>\r\n", CustomCommand->CommandType);
    
  switch(CustomCommand->CommandType) { 
    case BLE_CUSTOM_COMMAND_VOID:
      if(!strncmp((char *)CustomCommand->CommandName,"BleManagerReset",15)) {
          aci_gap_terminate(CurrentConnectionHandle,0x13 /* */);
          HAL_Delay(5000);
          needToResetBLE=1;
      }
    break;
    case BLE_CUSTOM_COMMAND_INTEGER:
      BLE_MANAGER_PRINTF("\tInt    Value: <%d>\r\n", CustomCommand->IntValue);
    break;
    case BLE_CUSTOM_COMMAND_ENUM_INTEGER:
      BLE_MANAGER_PRINTF("\tInt     Enum: <%d>\r\n", CustomCommand->IntValue);
    break;
    case BLE_CUSTOM_COMMAND_BOOLEAN:
      BLE_MANAGER_PRINTF("\tInt    Value: <%d>\r\n", CustomCommand->IntValue);
    break;
    case  BLE_CUSTOM_COMMAND_STRING:
      BLE_MANAGER_PRINTF("\tString Value: <%s>\r\n", CustomCommand->StringValue);
    break;
    case  BLE_CUSTOM_COMMAND_ENUM_STRING:
      BLE_MANAGER_PRINTF("\tString  Enum: <%s>\r\n", CustomCommand->StringValue);
    break;
  }
  
   if(!strncmp((char *)CustomCommand->CommandName,"ChangeCustomCommand",20)) {
     CustomCommandPageLevel=1;
     SendNewCustomCommandList();
   } else if(!strncmp((char *)CustomCommand->CommandName,"ComeBackCustomCommand",21)) {
     CustomCommandPageLevel=0;
     SendNewCustomCommandList();
   } else if(!strncmp((char *)CustomCommand->CommandName,"IntValue2",9)) {
     SendError("Example of Error");
   } else if(!strncmp((char *)CustomCommand->CommandName,"IntValue1",9)) {
     SendInfo("Example of Info");
   }
}

/**
 * @brief  Reads Sensor Config
 * @param  JSON_Array *JSON_SensorArray
 * @retval None
 */
static void ExtConfigReadSensorConfigCommandCallback(JSON_Array *JSON_SensorArray)
{
  
#define WRITE_BUFFER_SIZE_HTS221_H       (uint32_t)(256)
#define WRITE_BUFFER_SIZE_HTS221_T       (uint32_t)(256)
#define WRITE_BUFFER_SIZE_LPS22HH_P       (uint32_t)(1024)
#define WRITE_BUFFER_SIZE_LPS22HH_T       (uint32_t)(1024)
#define WRITE_BUFFER_SIZE_LSM6DSOX_A   (uint32_t)(16384)
#define WRITE_BUFFER_SIZE_LSM6DSOX_G   (uint32_t)(16384)
#define WRITE_BUFFER_SIZE_LSM6DSOX_MLC (uint32_t)(1024)
  
  COM_Sensor_t SensorHTS221,SensorLPS22HH,SensorLSM6DSOX;
  JSON_Value *tempJSON1;
  BLE_MANAGER_PRINTF("Received the Read Sensors Config Command\r\n");
  
  /* HTS221 SENSOR DESCRIPTOR */
  strcpy(SensorHTS221.sensorDescriptor.name, "HTS221");
  SensorHTS221.sensorDescriptor.nSubSensors = 2;
  SensorHTS221.sensorDescriptor.id=0;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  SensorHTS221.sensorDescriptor.subSensorDescriptor[0].id = 0;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_TEMP;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[0].dimensions = 1;
  strcpy(SensorHTS221.sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "tem");
  SensorHTS221.sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_FLOAT;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[0].ODR[0] = 1.0f;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[0].ODR[1] = 7.0f;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[0].ODR[2] = 12.5f;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[0].ODR[3] = COM_END_OF_LIST_FLOAT;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(SensorHTS221.sensorDescriptor.subSensorDescriptor[0].unit, "Celsius");
  SensorHTS221.sensorDescriptor.subSensorDescriptor[0].FS[0] = 120.0f;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  SensorHTS221.sensorStatus.subSensorStatus[0].isActive = 1;
  SensorHTS221.sensorStatus.subSensorStatus[0].FS = 120.0f;
  SensorHTS221.sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  SensorHTS221.sensorStatus.subSensorStatus[0].ODR = 12.5f;
  SensorHTS221.sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  SensorHTS221.sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  SensorHTS221.sensorStatus.subSensorStatus[0].samplesPerTimestamp = 50;
  SensorHTS221.sensorStatus.subSensorStatus[0].usbDataPacketSize = 16;
  SensorHTS221.sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_HTS221_T;
  SensorHTS221.sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  SensorHTS221.sensorStatus.subSensorStatus[0].ucfLoaded = 0;
  
  /* SUBSENSOR 1 DESCRIPTOR */
  SensorHTS221.sensorDescriptor.subSensorDescriptor[1].id = 1;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[1].sensorType = COM_TYPE_HUM;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[1].dimensions = 1;
  strcpy(SensorHTS221.sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[0], "hum");
  SensorHTS221.sensorDescriptor.subSensorDescriptor[1].dataType = DATA_TYPE_FLOAT;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[1].ODR[0] = 1.0f;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[1].ODR[1] = 7.0f;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[1].ODR[2] = 12.5f;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[1].ODR[3] = COM_END_OF_LIST_FLOAT;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[0] = 0;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[1] = 1000;
  strcpy(SensorHTS221.sensorDescriptor.subSensorDescriptor[1].unit, "%");
  SensorHTS221.sensorDescriptor.subSensorDescriptor[1].FS[0] = 100.0f;
  SensorHTS221.sensorDescriptor.subSensorDescriptor[1].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 1 STATUS */
  SensorHTS221.sensorStatus.subSensorStatus[1].isActive = 1;
  SensorHTS221.sensorStatus.subSensorStatus[1].FS = 100.0f;
  SensorHTS221.sensorStatus.subSensorStatus[1].sensitivity = 1.0f;
  SensorHTS221.sensorStatus.subSensorStatus[1].ODR = 12.5f;
  SensorHTS221.sensorStatus.subSensorStatus[1].measuredODR = 0.0f;
  SensorHTS221.sensorStatus.subSensorStatus[1].initialOffset = 0.0f;
  SensorHTS221.sensorStatus.subSensorStatus[1].samplesPerTimestamp = 50;
  SensorHTS221.sensorStatus.subSensorStatus[1].usbDataPacketSize = 16;
  SensorHTS221.sensorStatus.subSensorStatus[1].sdWriteBufferSize = WRITE_BUFFER_SIZE_HTS221_H;
  SensorHTS221.sensorStatus.subSensorStatus[1].comChannelNumber = -1;
  SensorHTS221.sensorStatus.subSensorStatus[1].ucfLoaded = 0;
  
  /* LPS22HH SENSOR DESCRIPTOR */
  strcpy(SensorLPS22HH.sensorDescriptor.name, "LPS22HH");
  SensorLPS22HH.sensorDescriptor.nSubSensors = 2;
  SensorLPS22HH.sensorDescriptor.id=1;
  
  /* SUBSENSOR 0 DESCRIPTOR */
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].id = 0;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_PRESS;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].dimensions = 1;
  strcpy(SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "prs");
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_FLOAT;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].ODR[0] = 1.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].ODR[1] = 10.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].ODR[2] = 25.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].ODR[3] = 50.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].ODR[4] = 75.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].ODR[5] = 100.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].ODR[6] = 200.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].ODR[7] = COM_END_OF_LIST_FLOAT;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].unit, "hPa");
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].FS[0] = 1260.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[0].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 0 STATUS */
  SensorLPS22HH.sensorStatus.subSensorStatus[0].isActive = 1;
  SensorLPS22HH.sensorStatus.subSensorStatus[0].FS = 1260.0f;
  SensorLPS22HH.sensorStatus.subSensorStatus[0].sensitivity = 1.0f;
  SensorLPS22HH.sensorStatus.subSensorStatus[0].ODR = 200.0f;
  SensorLPS22HH.sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  SensorLPS22HH.sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  SensorLPS22HH.sensorStatus.subSensorStatus[0].samplesPerTimestamp = 200;
  SensorLPS22HH.sensorStatus.subSensorStatus[0].usbDataPacketSize = 1600;
  SensorLPS22HH.sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_LPS22HH_P;
  SensorLPS22HH.sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  SensorLPS22HH.sensorStatus.subSensorStatus[0].ucfLoaded = 0;
  
  /* SUBSENSOR 1 DESCRIPTOR */
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].id = 1;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].sensorType = COM_TYPE_TEMP;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].dimensions = 1;
  strcpy(SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[0], "tem");
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].dataType = DATA_TYPE_FLOAT;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].ODR[0] = 1.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].ODR[1] = 10.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].ODR[2] = 25.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].ODR[3] = 50.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].ODR[4] = 75.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].ODR[5] = 100.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].ODR[6] = 200.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].ODR[7] = COM_END_OF_LIST_FLOAT;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[0] = 0;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[1] = 1000;
  strcpy(SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].unit, "Celsius");
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].FS[0] = 85.0f;
  SensorLPS22HH.sensorDescriptor.subSensorDescriptor[1].FS[1] = COM_END_OF_LIST_FLOAT;
  
  /* SUBSENSOR 1 STATUS */
  SensorLPS22HH.sensorStatus.subSensorStatus[1].isActive = 1;
  SensorLPS22HH.sensorStatus.subSensorStatus[1].FS = 85.0f;
  SensorLPS22HH.sensorStatus.subSensorStatus[1].sensitivity = 1.0f;
  SensorLPS22HH.sensorStatus.subSensorStatus[1].ODR = 200.0f;
  SensorLPS22HH.sensorStatus.subSensorStatus[1].measuredODR = 0.0f;
  SensorLPS22HH.sensorStatus.subSensorStatus[1].initialOffset = 0.0f;
  SensorLPS22HH.sensorStatus.subSensorStatus[1].samplesPerTimestamp = 200;
  SensorLPS22HH.sensorStatus.subSensorStatus[1].usbDataPacketSize = 1600;
  SensorLPS22HH.sensorStatus.subSensorStatus[1].sdWriteBufferSize = WRITE_BUFFER_SIZE_LPS22HH_T;
  SensorLPS22HH.sensorStatus.subSensorStatus[1].comChannelNumber = -1;
  SensorLPS22HH.sensorStatus.subSensorStatus[1].ucfLoaded = 0;
  
  
  /* SENSOR DESCRIPTOR */
  strcpy(SensorLSM6DSOX.sensorDescriptor.name, "LSM6DSOX");
  SensorLSM6DSOX.sensorDescriptor.nSubSensors = 3;
  SensorLSM6DSOX.sensorDescriptor.id=2;

  /* SUBSENSOR 0 DESCRIPTOR */
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].id = 0;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].sensorType = COM_TYPE_ACC;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].dimensions = 3;
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[0], "x");
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[1], "y");
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].dimensionsLabel[2], "z");
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].dataType = DATA_TYPE_INT16;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].ODR[0] = 12.5f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].ODR[1] = 26.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].ODR[2] = 52.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].ODR[3] = 104.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].ODR[4] = 208.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].ODR[5] = 417.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].ODR[6] = 833.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].ODR[7] = 1667.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].ODR[8] = 3333.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].ODR[9] = 6667.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].ODR[10] = COM_END_OF_LIST_FLOAT;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[0] = 0;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].samplesPerTimestamp[1] = 1000;
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].unit, "g");
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].FS[0] = 2.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].FS[1] = 4.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].FS[2] = 8.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].FS[3] = 16.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[0].FS[4] = COM_END_OF_LIST_FLOAT;

  /* SUBSENSOR 0 STATUS */
  SensorLSM6DSOX.sensorStatus.subSensorStatus[0].isActive = 1;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[0].FS = 16.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[0].sensitivity = 0.0000305f * SensorLSM6DSOX.sensorStatus.subSensorStatus[0].FS;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[0].ODR = 6667.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[0].measuredODR = 0.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[0].initialOffset = 0.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[0].samplesPerTimestamp = 1000;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[0].usbDataPacketSize = 2048;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[0].sdWriteBufferSize = WRITE_BUFFER_SIZE_LSM6DSOX_A;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[0].comChannelNumber = -1;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[0].ucfLoaded = 0;

    /* SUBSENSOR 1 DESCRIPTOR */
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].id = 1;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].sensorType = COM_TYPE_GYRO;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].dimensions = 3;
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[0], "x");
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[1], "y");
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].dimensionsLabel[2], "z");
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].dataType = DATA_TYPE_INT16;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].ODR[0] = 12.5f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].ODR[1] = 26.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].ODR[2] = 52.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].ODR[3] = 104.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].ODR[4] = 208.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].ODR[5] = 417.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].ODR[6] = 833.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].ODR[7] = 1667.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].ODR[8] = 3333.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].ODR[9] = 6667.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].ODR[10] = COM_END_OF_LIST_FLOAT;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[0] = 0;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].samplesPerTimestamp[1] = 1000;
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].unit, "mdps");
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].FS[0] = 125.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].FS[1] = 250.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].FS[2] = 500.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].FS[3] = 1000.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].FS[4] = 2000.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[1].FS[5] = COM_END_OF_LIST_FLOAT;

  /* SUBSENSOR 1 STATUS */
  SensorLSM6DSOX.sensorStatus.subSensorStatus[1].isActive = 1;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[1].FS = 2000.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[1].sensitivity = 0.035f * SensorLSM6DSOX.sensorStatus.subSensorStatus[1].FS;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[1].ODR = 6667.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[1].measuredODR = 0.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[1].initialOffset = 0.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[1].samplesPerTimestamp = 1000;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[1].usbDataPacketSize = 2048;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[1].sdWriteBufferSize = WRITE_BUFFER_SIZE_LSM6DSOX_G;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[1].comChannelNumber = -1;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[1].ucfLoaded = 0;

    /* SUBSENSOR 2 DESCRIPTOR */
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].id = 2;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].sensorType = COM_TYPE_MLC;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].dimensions = 8;
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[0], "1");
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[1], "2");
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[2], "3");
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[3], "4");
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[4], "5");
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[5], "6");
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[6], "7");
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].dimensionsLabel[7], "8");
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].dataType = DATA_TYPE_INT8;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].ODR[0] = 12.5f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].ODR[1] = 26.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].ODR[2] = 52.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].ODR[3] = 104.0f;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].ODR[4] = COM_END_OF_LIST_FLOAT;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].samplesPerTimestamp[0] = 0;
  SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].samplesPerTimestamp[1] = 1000;
  strcpy(SensorLSM6DSOX.sensorDescriptor.subSensorDescriptor[2].unit, "out");

  /* SUBSENSOR 2 STATUS */
  SensorLSM6DSOX.sensorStatus.subSensorStatus[2].isActive = 0;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[2].FS = 0.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[2].sensitivity = 1.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[2].ODR = 0.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[2].measuredODR = 0.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[2].initialOffset = 0.0f;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[2].samplesPerTimestamp = 1;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[2].usbDataPacketSize = 16;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[2].sdWriteBufferSize = WRITE_BUFFER_SIZE_LSM6DSOX_MLC;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[2].comChannelNumber = -1;
  SensorLSM6DSOX.sensorStatus.subSensorStatus[2].ucfLoaded = 0;
  
  //Add the sensors to the Array
  tempJSON1 = json_value_init_object();
  create_JSON_Sensor(&SensorHTS221, tempJSON1);
  json_array_append_value(JSON_SensorArray,tempJSON1);
  tempJSON1 = json_value_init_object();
  create_JSON_Sensor(&SensorLPS22HH, tempJSON1);
  json_array_append_value(JSON_SensorArray,tempJSON1);
  tempJSON1 = json_value_init_object();
  create_JSON_Sensor(&SensorLSM6DSOX, tempJSON1);
  json_array_append_value(JSON_SensorArray,tempJSON1);
  
}

/**
 * @brief  Custom commands definition
 * @param  JSON_Array *JSON_SensorArray
 * @retval None
 */
static void ExtConfigReadCustomCommandsCallback(JSON_Array *JSON_SensorArray)
{
  /* Clear the previous Costom Command List */
  ClearCustomCommandsList();
  
  if(CustomCommandPageLevel==0) {
  
    /* Add all the custom Commands */
    if(AddCustomCommand("IntValue1", //Name
                        BLE_CUSTOM_COMMAND_INTEGER, //Type
                        -100, //MIN
                        200,  //MAX
                        NULL, //Enum Int
                        NULL, //Enum String
                        NULL, //Description
                        JSON_SensorArray)) {
      BLE_MANAGER_PRINTF("Added Command <%s>\r\n","IntValue1");
    } else {
       BLE_MANAGER_PRINTF("Error Adding Command <%s>\r\n","IntValue1");
       return;
    }
    
    if(AddCustomCommand("IntValue2", //Name
                        BLE_CUSTOM_COMMAND_INTEGER, //Type
                        10, //MIN
                        3000,  //MAX
                        NULL, //Enum Int
                        NULL, //Enum String
                        NULL, //Description
                        JSON_SensorArray)) {
      BLE_MANAGER_PRINTF("Added Command <%s>\r\n","IntValue2");
    } else {
       BLE_MANAGER_PRINTF("Error Adding Command <%s>\r\n","IntValue2");
       return;
    }

    if(AddCustomCommand("BleManagerReset", //Name
                        BLE_CUSTOM_COMMAND_VOID, //Type
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN,  //MAX
                        NULL, //Enum Int
                        NULL, //Enum String
                        "Reset the Bluetooth", //Description
                        JSON_SensorArray)) {
      BLE_MANAGER_PRINTF("Added Command <%s>\r\n","Command1");
    } else {
       BLE_MANAGER_PRINTF("Error Adding Command <%s>\r\n","Command1");
       return;
    }
    
    if(AddCustomCommand("StringValue1", //Name
                        BLE_CUSTOM_COMMAND_STRING, //Type
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                        20,  //MAX
                        NULL, //Enum Int
                        NULL, //Enum String
                        NULL, //Description
                        JSON_SensorArray)) {
      BLE_MANAGER_PRINTF("Added Command <%s>\r\n","StringValue1");
    } else {
       BLE_MANAGER_PRINTF("Error Adding Command <%s>\r\n","StringValue1");
       return;
    }
    
    if(AddCustomCommand("BooleanValue", //Name
                        BLE_CUSTOM_COMMAND_BOOLEAN, //Type
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN,  //MAX
                        NULL, //Enum Int
                        NULL, //Enum String
                        "Example for Boolean", //Description
                        JSON_SensorArray)) {
      BLE_MANAGER_PRINTF("Added Command <%s>\r\n","BooleanValue");
    } else {
       BLE_MANAGER_PRINTF("Error Adding Command <%s>\r\n","BooleanValue");
       return;
    }
       
    if(AddCustomCommand("StringValue2", //Name
                        BLE_CUSTOM_COMMAND_STRING, //Type
                        4, //MIN
                        10,  //MAX
                        NULL, //Enum Int
                        NULL, //Enum String
                        "It's possible to add a  very very very very very very long description", //Description
                        JSON_SensorArray)) {
      BLE_MANAGER_PRINTF("Added Command <%s>\r\n","StringValue2");
    } else {
      BLE_MANAGER_PRINTF("Error Adding Command <%s>\r\n","StringValue2");
      return;
    }
    
    //Example of Enum String Custom Command
    {
      //The Last value should be NULL
      char *ValidStringValues[]={"Ciao", "Buona","Giornata",NULL};
      if(AddCustomCommand("StringEnum", //Name
                          BLE_CUSTOM_COMMAND_ENUM_STRING, //Type
                          BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                          BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN,  //MAX
                          NULL, //Enum Int
                          (void *)ValidStringValues, //Enum String
                          "Example of Enum String", //Description
                          JSON_SensorArray)) {
        BLE_MANAGER_PRINTF("Added Command <%s>\r\n","StringEnum");
      } else {
        BLE_MANAGER_PRINTF("Error Adding Command <%s>\r\n","StringEnum");
        return;
      }
    } 
    
    //Example of Enum Int Custom Command
    {
      //The Last value should be BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN
      int32_t ValidIntValues[]={-1,12,123,321,BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN};
      if(AddCustomCommand("IntEnum", //Name
                          BLE_CUSTOM_COMMAND_ENUM_INTEGER, //Type
                          BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                          BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN,  //MAX
                          (void *) ValidIntValues, //Enum Int
                          NULL, //Enum String
                          "Example of Enum Integer", //Description
                          JSON_SensorArray)) {
        BLE_MANAGER_PRINTF("Added Command <%s>\r\n","IntEnum");
      } else {
        BLE_MANAGER_PRINTF("Error Adding Command <%s>\r\n","IntEnum");
        return;
      }
    }
    
     if(AddCustomCommand("ChangeCustomCommand", //Name
                        BLE_CUSTOM_COMMAND_VOID, //Type
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN,  //MAX
                        NULL, //Enum Int
                        NULL, //Enum String
                        "Change the Custom Commands", //Description
                        JSON_SensorArray)) {
      BLE_MANAGER_PRINTF("Added Command <%s>\r\n","Command1");
    } else {
       BLE_MANAGER_PRINTF("Error Adding Command <%s>\r\n","Command1");
       return;
    }
    
    //Just one Example of one Invalid Command
    if(AddCustomCommand("ReadCert", //Name
                        BLE_CUSTOM_COMMAND_STRING, //Type
                        4, //MIN
                        10,  //MAX
                        NULL, //Enum Int
                        NULL, //Enum String
                        "Invalid Command...", //Description
                        JSON_SensorArray)) {
      BLE_MANAGER_PRINTF("Added Command <%s>\r\n","ReadCert");
    } else {
      BLE_MANAGER_PRINTF("Error Adding Command <%s>\r\n","ReadCert");
      return;//not mandatory... it's the last one
    }
  } else if(CustomCommandPageLevel==1) {
     if(AddCustomCommand("ComeBackCustomCommand", //Name
                        BLE_CUSTOM_COMMAND_VOID, //Type
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN,  //MAX
                        NULL, //Enum Int
                        NULL, //Enum String
                        "Come back to previous Custom Commands", //Description
                        JSON_SensorArray)) {
      BLE_MANAGER_PRINTF("Added Command <%s>\r\n","Command1");
    } else {
       BLE_MANAGER_PRINTF("Error Adding Command <%s>\r\n","Command1");
       return;
    }
    
    if(AddCustomCommand("StringValueNewLevel", //Name
                        BLE_CUSTOM_COMMAND_STRING, //Type
                        BLE_MANAGER_CUSTOM_COMMAND_VALUE_NAN, //MIN
                        20,  //MAX
                        NULL, //Enum Int
                        NULL, //Enum String
                        NULL, //Description
                        JSON_SensorArray)) {
      BLE_MANAGER_PRINTF("Added Command <%s>\r\n","StringValueNewLevel");
    } else {
       BLE_MANAGER_PRINTF("Error Adding Command <%s>\r\n","StringValue1");
       return;
    }
  }
}

/**
 * @brief  Callback Function for managing the DFU command
 * @param  None
 * @retval None
 */
static void ExtConfigRebootOnDFUModeCommandCallback(void)
{
  BLE_MANAGER_PRINTF("RebootOnDFUModeCommandCallback\r\n");
  
  /* Insert here the code for managing the received command */
  /* Reboot the board on DFU mode */
  //HAL_NVIC_SystemReset();
}

/**
 * @brief  Callback Function for answering to the UID command
 * @param  uint8_t **UID STM32 UID Return value
 * @retval None
 */
static void ExtExtConfigUidCommandCallback(uint8_t **UID)
{
  *UID = (uint8_t *)STM32_UUID;
}


/**
 * @brief  Callback Function for answering to Info command
 * @param  uint8_t *Answer Return String
 * @retval None
 */
static void ExtConfigInfoCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer,"STMicroelectronics %s:\n"
    "Version %c.%c.%c\n"
    "STM32L4R9ZI-SensorTile.box board\n"
    "(HAL %ld.%ld.%ld_%ld)\n"
    "Compiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
    " (IAR)",
#elif defined (__CC_ARM)
    " (KEIL)",
#elif defined (__GNUC__)
    " (STM32CubeIDE)",
#endif
    BLE_FW_PACKAGENAME,
    BLE_VERSION_FW_MAJOR,
    BLE_VERSION_FW_MINOR,
    BLE_VERSION_FW_PATCH,
    HAL_GetHalVersion() >>24,
    (HAL_GetHalVersion() >>16)&0xFF,
    (HAL_GetHalVersion() >> 8)&0xFF,
     HAL_GetHalVersion()      &0xFF,
     __DATE__,__TIME__);
}

/**
 * @brief  Callback Function for answering to Help command
 * @param  uint8_t *Answer Return String
 * @retval None
 */
static void ExtConfigHelpCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer,"Print out some help\nCiao\nLuca");
}

/**
 * @brief  Callback Function for answering to PowerStatus command
 * @param  uint8_t *Answer Return String
 * @retval None
 */
static void ExtConfigPowerStatusCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer,"Plug the Battery");
}
  
/**
 * @brief  Callback Function for answering to VersionFw command
 * @param  uint8_t *Answer Return String
 * @retval None
 */
static void ExtConfigVersionFwCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer,"%s_%s_%c.%c.%c",
      BLE_STM32_MICRO,
      BLE_FW_PACKAGENAME,
      BLE_VERSION_FW_MAJOR,
      BLE_VERSION_FW_MINOR,
      BLE_VERSION_FW_PATCH);
}

/**
 * @brief  Callback Function for managing the PowerOff command
 * @param  None
 * @retval None
 */
static void ExtConfigPowerOffCommandCallback(void)
{
  BLE_MANAGER_PRINTF("ExtConfigPowerOffCommandCallback\r\n");
  
  /* Insert here the code for managing the received command */
}

/**
 * @brief  Callback Function for managing the SetName command
 * @param  uint8_t *NewName
 * @retval None
 */
static void ExtConfigSetNameCommandCallback(uint8_t *NewName)
{ 
  BLE_MANAGER_PRINTF("New Board Name = <%s>\r\n", NewName);
  /* Change the Board Name */
  sprintf(BlueNRG_StackValue.BoardName,"%s",NewName);
}

/**
 * @brief  Callback Function for managing the SetTime command
 * @param  uint8_t *NewTime
 * @retval None
 */
static void ExtConfigSetTimeCommandCallback(uint8_t *NewTime)
{
  BLE_MANAGER_PRINTF("New Board Time= <%s>\r\n", NewTime);
  
  /* Insert here the code for changing the RTC time */
}

/**
 * @brief  Callback Function for managing the SetDate command
 * @param  uint8_t *NewDate
 * @retval None
 */
static void ExtConfigSetDateCommandCallback(uint8_t *NewDate)
{
  BLE_MANAGER_PRINTF("New Board Date= <%s>\r\n", NewDate);
  
  /* Insert here the code for changing the RTC Date */
}

/**
 * @brief  Callback Function for managing the SetWiFi command
 * @param  BLE_WiFi_CredAcc_t NewWiFiCred
 * @retval None
 */
static void ExtConfigSetWiFiCommandCallback(BLE_WiFi_CredAcc_t NewWiFiCred)
{
  BLE_MANAGER_PRINTF("NewWiFiCred=\r\n");
  BLE_MANAGER_PRINTF("\tSSID    = <%s>\r\n", NewWiFiCred.SSID);
  BLE_MANAGER_PRINTF("\tPassWd = <%s>\r\n", NewWiFiCred.PassWd);
  BLE_MANAGER_PRINTF("\tSecurity= <%s>\r\n", NewWiFiCred.Security);
  
  /* Insert here the code for changing the Wi-Fi Credential */
}

/**
 * @brief  Callback Function for managing the ReadCert command
 * @param  uint8_t *Certificate to register 
 * @retval None
 */
static void ExtConfigReadCertCommandCallback(uint8_t *Certificate)
{
  const char CertFromSTsafe[] = {
"-----BEGIN CERTIFICATE-----\r\n"
"MIIBjjCCATSgAwIBAgILAgnwIEAhzCJbATkwCgYIKoZIzj0EAwIwTzELMAkGA1UE\r\n"
"BhMCTkwxHjAcBgNVBAoMFVNUTWljcm9lbGVjdHJvbmljcyBudjEgMB4GA1UEAwwX\r\n"
"U1RNIFNUU0FGRS1BIFBST0QgQ0EgMDEwIBcNMjAwMjI2MDAwMDAwWhgPMjA1MDAy\r\n"
"MjYwMDAwMDBaMEYxCzAJBgNVBAYTAkZSMRswGQYDVQQKDBJTVE1pY3JvZWxlY3Ry\r\n"
"b25pY3MxGjAYBgNVBAMMEVNUU0FGRS1BMTEwIEVWQUwyMFkwEwYHKoZIzj0CAQYI\r\n"
"KoZIzj0DAQcDQgAEQCibQYjdHzn8yUyHPbZq1QUYEzSh0SrB2rkj/jDroUNqFkjF\r\n"
"d5mZ5ZxVjFz1mbZUvAIBrwvrT7XpOmVuMRzJRDAKBggqhkjOPQQDAgNIADBFAiB5\r\n"
"yNIKxSMcazW0IvclwZyeo83pVC1Q3tIKSIZJZbP2EgIhAOx7kYZnLUlyuckX0HU4\r\n"
"Tel4Ayt9RewWGxHPZIo4K+JR\r\n"
"-----END CERTIFICATE-----\r\n"
};
                      
  sprintf((char *)Certificate,"%s",CertFromSTsafe);
}

/**
 * @brief  Callback Function for managing the ChangePin command
 * @param  uint32_t NewPin
 * @retval None
 */
static void ExtConfigChangePinCommandCallback(uint32_t NewPin)
{
   BLE_MANAGER_PRINTF("New Board Pin= <%d>\r\n", NewPin);

   BlueNRG_StackValue.SecurePIN=NewPin;
}

/**
 * @brief  Callback Function for managing the ClearDB command
 * @param  None
 * @retval None
 */
static void ExtConfigClearDBCommandCallback()
{
  BLE_MANAGER_PRINTF("ExtConfigClearDBCommandCallback\r\n");
  NeedToClearSecureDB=1;
}

/**
 * @brief  Callback Function for managing the SetCert command
 * @param  uint8_t *Certificate registerd certificate
 * @retval None
 */
static void ExtConfigSetCertCommandCallback(uint8_t *Certificate)
{ 
  BLE_MANAGER_PRINTF("Certificate From Dashboard= <%s>\r\n", Certificate);
}

