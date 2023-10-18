/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    BLE_Implementation.c
  * @author  System Research & Applications Team - Catania Lab.
  * @brief   BLE Implementation header template file.
  *          This file should be copied to the application folder and renamed
  *          to BLE_Implementation.c.
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

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "BLE_Manager.h"
#include "main.h"

__weak void BLE_SetCustomAdvertiseData(uint8_t *manuf_data);
__weak void DisconnectionCompletedFunction(void);
__weak void ConnectionCompletedFunction(uint16_t ConnectionHandle, uint8_t Address_Type, uint8_t addr[6]);
__weak void SetBoardName(void);

__weak void AttrModConfigFunction(uint8_t * att_data, uint8_t data_length);
__weak void PairingCompletedFunction(uint8_t PairingStatus);
__weak void SetConnectableFunction(uint8_t *ManufData);
__weak void AciGattTxPoolAvailableEventFunction(void);
__weak void HardwareErrorEventHandlerFunction(uint8_t Hardware_Code);

__weak uint32_t DebugConsoleParsing(uint8_t * att_data, uint8_t data_length);
__weak void WriteRequestConfigFunction(uint8_t * att_data, uint8_t data_length);

/**********************************************************************************************
 * Callback functions prototypes to manage the extended configuration characteristic commands *
 **********************************************************************************************/
__weak void ExtExtConfigUidCommandCallback(uint8_t **UID);
__weak void ExtConfigVersionFwCommandCallback(uint8_t *Answer);
__weak void ExtConfigInfoCommandCallback(uint8_t *Answer);
__weak void ExtConfigHelpCommandCallback(uint8_t *Answer);

__weak void ExtConfigSetNameCommandCallback(uint8_t *NewName);

__weak void ReadRequestEnvFunction(int32_t *Press,uint16_t *Hum,int16_t *Temp1,int16_t *Temp2);

/*************************************************************
 * Callback functions prototypes to manage the notify events *
 *************************************************************/

__weak void NotifyEventAudioLevel(BLE_NotifyEvent_t Event);

__weak void NotifyEventEnv(BLE_NotifyEvent_t Event);

__weak void NotifyEventInertial(BLE_NotifyEvent_t Event);

__weak void NotifyEventActRec(BLE_NotifyEvent_t Event);

__weak void NotifyEventAudioSourceLocalization(BLE_NotifyEvent_t Event);

__weak void NotifyEventECompass(BLE_NotifyEvent_t Event);

__weak void NotifyEventSensorFusion(BLE_NotifyEvent_t Event);

/* Private variables ------------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private functions ---------------------------------------------------------*/

/** @brief Initialize the BlueNRG stack and services
  * @param  None
  * @retval None
  */
void BluetoothInit(void)
{
  /* BlueNRG stack setting */
  BLE_StackValue.ConfigValueOffsets                   = CONFIG_VALUE_OFFSETS;
  BLE_StackValue.ConfigValuelength                    = CONFIG_VALUE_LENGTH;
  BLE_StackValue.GAP_Roles                            = GAP_ROLES;
  BLE_StackValue.IO_capabilities                      = IO_CAPABILITIES;
  BLE_StackValue.AuthenticationRequirements           = BONDING;
  BLE_StackValue.MITM_ProtectionRequirements          = AUTHENTICATION_REQUIREMENTS;
  BLE_StackValue.SecureConnectionSupportOptionCode    = SECURE_CONNECTION_SUPPORT_OPTION_CODE;
  BLE_StackValue.SecureConnectionKeypressNotification = SECURE_CONNECTION_KEYPRESS_NOTIFICATION;

  /* Use BLE Random Address */
  BLE_StackValue.OwnAddressType = ADDRESS_TYPE;

  /* Set the BLE Board Name */
  SetBoardName();

  /* En_High_Power Enable High Power mode.
     High power mode should be enabled only to reach the maximum output power.
     Values:
     - 0x00: Normal Power
     - 0x01: High Power */
  BLE_StackValue.EnableHighPowerMode= ENABLE_HIGH_POWER_MODE;

  /* Values: 0x00 ... 0x31 - The value depends on the device */
  BLE_StackValue.PowerAmplifierOutputLevel = POWER_AMPLIFIER_OUTPUT_LEVEL;

  /* BlueNRG services setting */
  BLE_StackValue.EnableConfig    = ENABLE_CONFIG;
  BLE_StackValue.EnableConsole   = ENABLE_CONSOLE;
  BLE_StackValue.EnableExtConfig = ENABLE_EXT_CONFIG;

  /* For Enabling the Secure Connection */
  BLE_StackValue.EnableSecureConnection = ENABLE_SECURE_CONNECTION;
  /* Default Secure PIN */
  BLE_StackValue.SecurePIN = SECURE_PIN;
  /* For creating a Random Secure PIN */
  BLE_StackValue.EnableRandomSecurePIN = ENABLE_RANDOM_SECURE_PIN;

  /* Advertising policy for filtering (white list related) */
  BLE_StackValue.AdvertisingFilter = ADVERTISING_FILTER;

  if(BLE_StackValue.EnableSecureConnection) {
    /* Using the Secure Connection, the Rescan should be done by BLE chip */
    BLE_StackValue.ForceRescan =0;
  } else {
    BLE_StackValue.ForceRescan =1;
  }

  InitBleManager();
}

/**
 * @brief  Set Board Name.
 * @param  None
 * @retval None
 */
__weak void SetBoardName(void)
{
  sprintf(BLE_StackValue.BoardName,"%s%c%c%c","BLEM",
          BLE_VERSION_FW_MAJOR,
          BLE_VERSION_FW_MINOR,
          BLE_VERSION_FW_PATCH);
}

/**
 * @brief  Custom Service Initialization.
 * @param  None
 * @retval None
 */
void BLE_InitCustomService(void) {

  /* Define Custom Function for Connection Completed */
  CustomConnectionCompleted = ConnectionCompletedFunction;

  /* Define Custom Function for Disconnection Completed */
  CustomDisconnectionCompleted = DisconnectionCompletedFunction;

  /* Define Custom Function for Attribute Modify Config */
  CustomAttrModConfigCallback = AttrModConfigFunction;

  /* Define Custom Function for Pairing Completed */
  CustomPairingCompleted = PairingCompletedFunction;

  /* Define Custom Function for Set Connectable */
  CustomSetConnectable = SetConnectableFunction;

  /* Define Custom Function for Aci Gatt Tx Pool Available Event */
  CustomAciGattTxPoolAvailableEvent = AciGattTxPoolAvailableEventFunction;

  /* Define Custom Function for Hardware Error Event Handler */
  CustomHardwareErrorEventHandler = HardwareErrorEventHandlerFunction;

  /* Define Custom Function for Debug Console Command parsing */
  CustomDebugConsoleParsingCallback = DebugConsoleParsing;

  /* Define Custom Command for Parsing Write on Config Char */
  CustomWriteRequestConfigCallback = WriteRequestConfigFunction;

  /**************************************************************************************
   * Callback functions to manage the notify events and write request for each features *
   **************************************************************************************/

  CustomNotifyEventAudioLevel=                  NotifyEventAudioLevel;

  CustomNotifyEventEnv=                         NotifyEventEnv;

  CustomNotifyEventInertial=                    NotifyEventInertial;

  CustomNotifyEventActRec=                      NotifyEventActRec;

  CustomNotifyEventAudioSourceLocalization=     NotifyEventAudioSourceLocalization;

  CustomNotifyECompass=                         NotifyEventECompass;

  CustomNotifyEventSensorFusion=                NotifyEventSensorFusion;

  /***********************************************************************************
   * Callback functions to manage the extended configuration characteristic commands *
   ***********************************************************************************/
  CustomExtConfigUidCommandCallback  = ExtExtConfigUidCommandCallback;
  CustomExtConfigVersionFwCommandCallback = ExtConfigVersionFwCommandCallback;
  CustomExtConfigInfoCommandCallback = ExtConfigInfoCommandCallback;
  CustomExtConfigHelpCommandCallback = ExtConfigHelpCommandCallback;

  CustomExtConfigSetNameCommandCallback = ExtConfigSetNameCommandCallback;

  /**
  * For each features, user can assign here the pointer at the function for the read request data.
  * For example for the environmental features:
  *
  * CustomReadRequestEnv = ReadRequestEnvFunction;
  *
  * User can define and insert in the BLE_Implementation.c source code the functions for the read request data
  * ReadRequestEnvFunction function is already defined.
  *
  */

  /* Define Custom Function for Read Request Environmental Data */
  CustomReadRequestEnv = ReadRequestEnvFunction;

  /**
  * User can added here the custom service initialization for the selected BLE features.
  * For example for the environmental features:
  *
  * //BLE_InitEnvService(PressEnable,HumEnable,NumTempEnabled)
  * BleManagerAddChar(BleCharPointer= BLE_InitEnvService(1, 1, 1));
  */

  /* Characteristc allocation for audio level features */
  BleManagerAddChar(BLE_InitAudioLevelService(AUDIO_CHANNELS_NUMBER));

  /* Characteristc allocation for environmental features */
  /* BLE_InitEnvService(PressEnable,HumEnable,NumTempEnabled) */
  BleManagerAddChar(BLE_InitEnvService(ENABLE_ENV_PRESSURE_DATA, ENABLE_ENV_HUMIDITY_DATA, ENABLE_ENV_TEMPERATURE_DATA));

  /* Characteristc allocation for inertial features */
  /* BLE_InitInertialService(AccEnable,GyroEnable,MagEnabled) */
  BleManagerAddChar(BLE_InitInertialService(ENABLE_ACC_DATA,ENABLE_GYRO_DATA,ENABLE_MAG_DATA));

  /* Characteristc allocation for activity recognition features */
  BleManagerAddChar(BLE_InitActRecService());

  /* Characteristc allocation for audio source localization */
  BleManagerAddChar(BLE_InitAudioSourceLocalizationService());

  /* Characteristc allocation for E-Compass features */
  BleManagerAddChar(BLE_InitECompassService());

  /* Characteristc allocation for sensor fusion features */
  BleManagerAddChar(BLE_InitSensorFusionService(NUMBER_OF_QUATERNION));

}

/**
 * @brief  Set Custom Advertize Data.
 * @param  uint8_t *manuf_data: Advertize Data
 * @retval None
 */
__weak void BLE_SetCustomAdvertiseData(uint8_t *manuf_data)
{
#ifndef BLE_MANAGER_SDKV2
  /**
  * For only SDKV1, user can add here the custom advertize data setting for the selected BLE features.
  * For example for the environmental features:
  *
  * BLE_SetCustomEnvAdvertizeData(manuf_data);
  */

#else /* BLE_MANAGER_SDKV2 */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD1]=0xFF; /* Custom Firmware */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD2]=0x00;
  manuf_data[BLE_MANAGER_CUSTOM_FIELD3]=0x00;
  manuf_data[BLE_MANAGER_CUSTOM_FIELD4]=0x00;
#endif /* BLE_MANAGER_SDKV2 */
}

/**
* @brief  This function makes the parsing of the Debug Console
* @param  uint8_t *att_data attribute data
* @param  uint8_t data_length length of the data
* @retval uint32_t SendBackData true/false
*/
__weak uint32_t DebugConsoleParsing(uint8_t * att_data, uint8_t data_length)
{
  /* By default Answer with the same message received */
  uint32_t SendBackData =1;

  /* Help Command */
  if(!strncmp("help",(char *)(att_data),4))
  {
    /* Print Legend */
    SendBackData=0;

    BytesToWrite =sprintf((char *)BufferToWrite,"Command:\r\n"
      "info-> System Info\r\n"
      "uid-> STM32 UID value\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  }
  else if(!strncmp("info",(char *)(att_data),4))
  {
    SendBackData=0;

    BytesToWrite =sprintf((char *)BufferToWrite,"\r\nSTMicroelectronics %s:\r\n"
        "\tVersion %c.%c.%c\r\n"
        "\tSTM32F4xx MCU Family Name"
         "\r\n",
         BLE_FW_PACKAGENAME,
         BLE_VERSION_FW_MAJOR,BLE_VERSION_FW_MINOR,BLE_VERSION_FW_PATCH);

    Term_Update(BufferToWrite,BytesToWrite);

    BytesToWrite =sprintf((char *)BufferToWrite,"\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n",
#elif defined (__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) /* For ARM Compiler 5 and 6 */
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
  }
  else if((att_data[0]=='u') & (att_data[1]=='i') & (att_data[2]=='d'))
  {
    /* Write back the STM32 UID */
    uint8_t *uid = (uint8_t *)BLE_STM32_UUID;
    uint32_t MCU_ID = BLE_STM32_MCU_ID[0]&0xFFF;
    BytesToWrite =sprintf((char *)BufferToWrite,"%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X_%.3lX\r\n",
                          uid[ 3],uid[ 2],uid[ 1],uid[ 0],
                          uid[ 7],uid[ 6],uid[ 5],uid[ 4],
                          uid[11],uid[ 10],uid[9],uid[8],
                          MCU_ID);
    Term_Update(BufferToWrite,BytesToWrite);
    SendBackData=0;
  }

  /* NOTE: This function Should not be modified, when the callback is needed,
           the DebugConsoleParsing could be implemented in the user file
   */

  return SendBackData;
}

/**
 * @brief  Callback Function for Environmental read request.
 * @param  int32_t *Press Pressure Value
 * @param  uint16_t *Hum Humidity Value
 * @param  int16_t *Temp1 Temperature Number 1
 * @param  int16_t *Temp2 Temperature Number 2
 * @retval None
 */
__weak void ReadRequestEnvFunction(int32_t *Press,uint16_t *Hum,int16_t *Temp1,int16_t *Temp2)
{
  /* NOTE: Insert here the function to read the environmental data */
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None
 * @retval None
 */
__weak void DisconnectionCompletedFunction(void)
{
  BLE_MANAGER_PRINTF("Call to DisconnectionCompletedFunction\r\n");
  BLE_MANAGER_DELAY(100);

  /* NOTE: This function Should not be modified, when the callback is needed,
           the DisconnectionCompletedFunction could be implemented in the user file
   */
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint16_t ConnectionHandle
 * @param  uint8_t Address_Type
 * @param  uint8_t addr[6]
 * @retval None
 */
__weak void ConnectionCompletedFunction(uint16_t ConnectionHandle, uint8_t Address_Type, uint8_t addr[6])
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Address_Type);
  UNUSED(ConnectionHandle);
  UNUSED(addr);

  /* NOTE: This function Should not be modified, when the callback is needed,
           the ConnectionCompletedFunction could be implemented in the user file
   */

  BLE_MANAGER_PRINTF("Call to ConnectionCompletedFunction\r\n");
  BLE_MANAGER_DELAY(100);
}

/**
 * @brief  This function is called when there is a change on the gatt attribute.
 * @param  None
 * @retval None
 */
__weak void AttrModConfigFunction(uint8_t * att_data, uint8_t data_length)
{
  BLE_MANAGER_PRINTF("Call to AttrModConfigFunction\r\n");
  BLE_MANAGER_DELAY(100);

  /* NOTE: This function Should not be modified, when the callback is needed,
           the AttrModConfigFunction could be implemented in the user file
   */
}

/**
 * @brief  This function is called when the pairing process has completed successfully
 *         or a pairing procedure timeout has occurred or the pairing has failed.
 * @param  uint8_t PairingStatus
 * @retval None
 */
__weak void PairingCompletedFunction(uint8_t PairingStatus)
{
  BLE_MANAGER_PRINTF("Call to PairingCompletedFunction\r\n");
  BLE_MANAGER_DELAY(100);

  /* NOTE: This function Should not be modified, when the callback is needed,
           the PairingCompletedFunction could be implemented in the user file
   */
}

/**
 * @brief  This function is called when the device is put in connectable mode.
 * @param  uint8_t *ManufData Filling Manufacter Advertise data
 * @retval None
 */
__weak void SetConnectableFunction(uint8_t *ManufData)
{
  BLE_MANAGER_PRINTF("Call to SetConnectableFunction\r\n");
  BLE_MANAGER_DELAY(100);

  /* NOTE: This function Should not be modified, when the callback is needed,
           the SetConnectableFunction could be implemented in the user file
   */
}

/**
 * @brief  This function is called when bluetooth congestion buffer occurs
 *         or a pairing procedure timeout has occurred or the pairing has failed.
 * @param  None
 * @retval None
 */
__weak void AciGattTxPoolAvailableEventFunction(void)
{
  BLE_MANAGER_PRINTF("Call to AciGattTxPoolAvailableEventFunction\r\n");
  BLE_MANAGER_DELAY(100);

  /* NOTE: This function Should not be modified, when the callback is needed,
           the AciGattTxPoolAvailableEventFunction could be implemented in the user file
   */
}

/**
 * @brief  This event is used to notify the Host that a hardware failure has occurred in the Controller.
 * @param  uint8_t Hardware_Code Hardware Error Event code.
 * @retval None
 */
__weak void HardwareErrorEventHandlerFunction(uint8_t Hardware_Code)
{
  BLE_MANAGER_PRINTF("Call to HardwareErrorEventHandlerFunction\r\n");
  BLE_MANAGER_DELAY(100);

  /* NOTE: This function Should not be modified, when the callback is needed,
           the HardwareErrorEventHandlerFunction could be implemented in the user file
   */
}

/**
* @brief  Callback Function for Config write request.
* @param uint8_t *att_data attribute data
* @param uint8_t data_length length of the data
* @retval None
*/
__weak void WriteRequestConfigFunction(uint8_t * att_data, uint8_t data_length)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(att_data);
  UNUSED(data_length);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the WriteRequestConfigFunction could be implemented in the user file
   */
}

/**************************************************
 * Callback functions to manage the notify events *
 **************************************************/

/**
 * @brief  Callback Function for Un/Subscription Feature
 * @param  BLE_NotifyEvent_t Event Sub/Unsub
 * @retval None
 */
__weak void NotifyEventAudioLevel(BLE_NotifyEvent_t Event)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Event);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the NotifyEventAudioLevel could be implemented in the user file
   */
}

/**
 * @brief  Callback Function for Un/Subscription Feature
 * @param  BLE_NotifyEvent_t Event Sub/Unsub
 * @retval None
 */
__weak void NotifyEventEnv(BLE_NotifyEvent_t Event)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Event);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the NotifyEventEnv could be implemented in the user file
   */
}

/**
 * @brief  Callback Function for Un/Subscription Feature
 * @param  BLE_NotifyEvent_t Event Sub/Unsub
 * @retval None
 */
__weak void NotifyEventInertial(BLE_NotifyEvent_t Event)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Event);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the NotifyEventInertial could be implemented in the user file
   */
}

/**
 * @brief  Callback Function for Un/Subscription Feature
 * @param  BLE_NotifyEvent_t Event Sub/Unsub
 * @retval None
 */
__weak void NotifyEventActRec(BLE_NotifyEvent_t Event)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Event);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the NotifyEventActRec could be implemented in the user file
   */
}

/**
 * @brief  Callback Function for Un/Subscription Feature
 * @param  BLE_NotifyEvent_t Event Sub/Unsub
 * @retval None
 */
__weak void NotifyEventAudioSourceLocalization(BLE_NotifyEvent_t Event)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Event);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the NotifyEventAudioSourceLocalization could be implemented in the user file
   */
}

/**
 * @brief  Callback Function for Un/Subscription Feature
 * @param  BLE_NotifyEvent_t Event Sub/Unsub
 * @retval None
 */
__weak void NotifyEventECompass(BLE_NotifyEvent_t Event)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Event);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the NotifyEventECompass could be implemented in the user file
   */
}

/**
 * @brief  Callback Function for Un/Subscription Feature
 * @param  BLE_NotifyEvent_t Event Sub/Unsub
 * @retval None
 */
__weak void NotifyEventSensorFusion(BLE_NotifyEvent_t Event)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Event);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the NotifyEventSensorFusion could be implemented in the user file
   */
}

/***********************************************************************************
 * Callback functions to manage the extended configuration characteristic commands *
 ***********************************************************************************/
/**
 * @brief  Callback Function for answering to the UID command
 * @param  uint8_t **UID STM32 UID Return value
 * @retval None
 */
__weak void ExtExtConfigUidCommandCallback(uint8_t **UID)
{
#ifdef BLE_STM32_UUID
  *UID = (uint8_t *)BLE_STM32_UUID;
#endif /* BLE_STM32_UUID */

  /* NOTE: This function Should not be modified, when the callback is needed,
           the ExtExtConfigUidCommandCallback could be implemented in the user file
		   for managing the received command
   */
}

/**
 * @brief  Callback Function for answering to VersionFw command
 * @param  uint8_t *Answer Return String
 * @retval None
 */
__weak void ExtConfigVersionFwCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer,"%s_%s_%c.%c.%c",
      BLE_STM32_MICRO,
      BLE_FW_PACKAGENAME,
      BLE_VERSION_FW_MAJOR,
      BLE_VERSION_FW_MINOR,
      BLE_VERSION_FW_PATCH);

  /* NOTE: This function Should not be modified, when the callback is needed,
           the ExtConfigVersionFwCommandCallback could be implemented in the user file
		   for managing the received command
   */
}

/**
 * @brief  Callback Function for answering to Info command
 * @param  uint8_t *Answer Return String
 * @retval None
 */
__weak void ExtConfigInfoCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer,"STMicroelectronics %s:\n"
    "Version %c.%c.%c\n"
    "(HAL %ld.%ld.%ld_%ld)\n"
    "Compiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
    " (IAR)",
#elif defined (__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) /* For ARM Compiler 5 and 6 */
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

  /* NOTE: This function Should not be modified, when the callback is needed,
           the ExtConfigInfoCommandCallback could be implemented in the user file
		   for managing the received command
   */
}

/**
 * @brief  Callback Function for answering to Help command
 * @param  uint8_t *Answer Return String
 * @retval None
 */
__weak void ExtConfigHelpCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer,"List of available command:\n"
                         "1) Board Report\n"
                         "- STM32 UID\n"
                         "- Version Firmware\n"
                         "- Info\n"
                         "- Help\n\n");

  /* NOTE: This function Should not be modified, when the callback is needed,
           the ExtConfigHelpCommandCallback could be implemented in the user file
		   for managing the received command
   */
}

/**
 * @brief  Callback Function for managing the SetName command
 * @param  uint8_t *NewName
 * @retval None
 */
__weak void ExtConfigSetNameCommandCallback(uint8_t *NewName)
{
  BLE_MANAGER_PRINTF("New Board Name = <%s>\r\n", NewName);
  /* Change the Board Name */
  sprintf(BLE_StackValue.BoardName,"%s",NewName);

  /* NOTE: This function Should not be modified, when the callback is needed,
           the ExtConfigSetNameCommandCallback could be implemented in the user file
		   for managing the received command
   */
}
