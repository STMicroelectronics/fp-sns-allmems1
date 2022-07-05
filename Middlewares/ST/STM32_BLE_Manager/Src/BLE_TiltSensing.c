/**
  ******************************************************************************
  * @file    BLE_TiltSensing.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.1.0
  * @date    23-Dec-2021
  * @brief   Add Tilt Sensing service using vendor specific profiles.
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
#include "BLE_ManagerCommon.h"

/* Private define ------------------------------------------------------------*/
#define COPY_TILT_SENSING_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x0D,0x00,0x02,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#define TILT_SENSING_ADVERTIZE_DATA_POSITION  18

/* Exported variables --------------------------------------------------------*/
BLE_NotifyEnv_t BLE_TiltSensing_NotifyEvent = BLE_NOTIFY_NOTHING;
CustomReadRequestTiltSensing_t CustomReadRequestTiltSensing=NULL;

/* Private variables ---------------------------------------------------------*/
/* Data structure pointer for Tilt Sensing service */
static BleCharTypeDef BleTiltSensing;

/* Private functions ---------------------------------------------------------*/
static void AttrMod_Request_TiltSensing(void *BleCharPointer,uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data);
#ifndef BLUENRG_LP
static void Read_Request_TiltSensing(void *BleCharPointer,uint16_t handle);
#else /* BLUENRG_LP */
static void Read_Request_TiltSensing(void *BleCharPointer,
                             uint16_t handle,
                             uint16_t Connection_Handle,
                             uint8_t Operation_Type,
                             uint16_t Attr_Val_Offset,
                             uint8_t Data_Length,
                             uint8_t Data[]);
#endif /* BLUENRG_LP */

/**
 * @brief  Init Tilt Sensing service
 * @param  None
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for Tilt Sensing service
 */
BleCharTypeDef* BLE_InitTiltSensingService(void)
{
  /* Data structure pointer for BLE service */
  BleCharTypeDef *BleCharPointer;

  /* Init data structure pointer for Tilt Sensing info service */
  BleCharPointer = &BleTiltSensing;
  memset(BleCharPointer,0,sizeof(BleCharTypeDef));  
  BleCharPointer->AttrMod_Request_CB = AttrMod_Request_TiltSensing;
  BleCharPointer->Read_Request_CB= Read_Request_TiltSensing;
  COPY_TILT_SENSING_CHAR_UUID((BleCharPointer->uuid));
  BleCharPointer->Char_UUID_Type =UUID_TYPE_128;
  BleCharPointer->Char_Value_Length=2+12; /* 2 byte timestamp, 12 byte tilt sensing measurement */
  BleCharPointer->Char_Properties = ((uint8_t)CHAR_PROP_NOTIFY) | ((uint8_t)CHAR_PROP_READ);
  BleCharPointer->Security_Permissions=ATTR_PERMISSION_NONE;
  BleCharPointer->GATT_Evt_Mask=GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;
  BleCharPointer->Enc_Key_Size=16;
  BleCharPointer->Is_Variable=0;
  
  if(CustomReadRequestTiltSensing == NULL) {
    BLE_MANAGER_PRINTF("Warning: Read request Tilt Sensing function not defined\r\n");
  }
  
  BLE_MANAGER_PRINTF("BLE Tilt Sensing features ok\r\n");
  
  return BleCharPointer;
}

#ifndef BLE_MANAGER_SDKV2
/**
 * @brief  Setting Tilt Sensing Advertise Data
 * @param  uint8_t *manuf_data: Advertise Data
 * @retval None
 */
void BLE_SetTiltSensingAdvertizeData(uint8_t *manuf_data)
{
  manuf_data[TILT_SENSING_ADVERTIZE_DATA_POSITION] |= 0x0DU;
}
#endif /* BLE_MANAGER_SDKV2 */

/**
 * @brief  Update Tilt Sensing characteristic
 * @param  BLE_ANGLES_output_t TiltSensingMeasure Tilt Sensing Recognized
 * @retval tBleStatus   Status
 */
tBleStatus BLE_TiltSensingUpdate(BLE_ANGLES_output_t TiltSensingMeasure)
{  
  tBleStatus ret;
  
  uint8_t buff[2 + 12];
  uint8_t BuffPos;
  
  float TempFloat;
  uint32_t *TempBuff = (uint32_t *) & TempFloat;

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  BuffPos= 2;
  
  TempFloat = TiltSensingMeasure.AnglesArray[2];
  STORE_LE_32(&buff[BuffPos], *TempBuff);
  BuffPos += ((uint8_t)4);
  
  TempFloat = TiltSensingMeasure.AnglesArray[0];
  STORE_LE_32(&buff[BuffPos], *TempBuff);
  BuffPos += ((uint8_t)4);
  
  TempFloat = TiltSensingMeasure.AnglesArray[1];
  STORE_LE_32(&buff[BuffPos], *TempBuff);
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(&BleTiltSensing, 0, 2+12, buff);

  if (ret != (tBleStatus)BLE_STATUS_SUCCESS){
    if(BLE_StdErr_Service==BLE_SERV_ENABLE){
      BytesToWrite = (uint8_t)sprintf((char *)BufferToWrite, "Error Updating Tilt Sensing Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      BLE_MANAGER_PRINTF("Error Updating Tilt Sensing Char\r\n");
    }
  }
  return ret;
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 *         With this function it's possible to understand if Tilt Sensing is subscribed or not to the one service
 * @param  void *VoidCharPointer
 * @param  uint16_t attr_handle Handle of the attribute
 * @param  uint16_t Offset: (SoC mode) the offset is never used and it is always 0. Network coprocessor mode: 
 *                          - Bits 0-14: offset of the reported value inside the attribute.
 *                          - Bit 15: if the entire value of the attribute does not fit inside a single ACI_GATT_ATTRIBUTE_MODIFIED_EVENT event,
 *                            this bit is set to 1 to notify that other ACI_GATT_ATTRIBUTE_MODIFIED_EVENT events will follow to report the remaining value.                  
 * @param  uint8_t data_length length of the data
 * @param  uint8_t *att_data attribute data
 * @retval None
 */
static void AttrMod_Request_TiltSensing(void *VoidCharPointer, uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{
  if (att_data[0] == 01U) {
    BLE_TiltSensing_NotifyEvent= BLE_NOTIFY_SUB;
  } else if (att_data[0] == 0U){
    BLE_TiltSensing_NotifyEvent= BLE_NOTIFY_UNSUB;
  }
 
#if (BLE_DEBUG_LEVEL>1)
 if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
   BytesToWrite =(uint8_t)sprintf((char *)BufferToWrite,"--->Tilt Sensing=%s\n", (BLE_TiltSensing_NotifyEvent == BLE_NOTIFY_SUB) ? " ON" : " OFF");
   Term_Update(BufferToWrite,BytesToWrite);
 } else {
   BLE_MANAGER_PRINTF("--->Tilt Sensing=%s", (BLE_TiltSensing_NotifyEvent == BLE_NOTIFY_SUB) ? " ON\r\n" : " OFF\r\n");
 }
#endif
}

/**
 * @brief  This event is given when a read request is received by the server from the client.
 * @param  void *VoidCharPointer
 * @param  uint16_t handle Handle of the attribute
 * @retval None
 */
#ifndef BLUENRG_LP
static void Read_Request_TiltSensing(void *VoidCharPointer,uint16_t handle)
{
  if(CustomReadRequestTiltSensing != NULL) {
    BLE_ANGLES_output_t TiltSensingMeasure;
    CustomReadRequestTiltSensing(&TiltSensingMeasure);
    BLE_TiltSensingUpdate(TiltSensingMeasure);
   } else {
    BLE_MANAGER_PRINTF("\r\n\nRead request Tilt Sensing function not defined\r\n\n");
  }
}
#else /* BLUENRG_LP */
static void Read_Request_TiltSensing(void *BleCharPointer,
                                     uint16_t handle,
                                     uint16_t Connection_Handle,
                                     uint8_t Operation_Type,
                                     uint16_t Attr_Val_Offset,
                                     uint8_t Data_Length,
                                     uint8_t Data[])
{
  tBleStatus ret;
  if(CustomReadRequestTiltSensing != NULL) {
    BLE_ANGLES_output_t TiltSensingMeasure;
    uint8_t buff[2 + 12];
    uint8_t BuffPos;
    float TempFloat;
    uint32_t *TempBuff = (uint32_t *) & TempFloat;

    CustomReadRequestTiltSensing(&TiltSensingMeasure);
    
    STORE_LE_16(buff  ,(HAL_GetTick()>>3));
    BuffPos= 2;
    
    TempFloat = TiltSensingMeasure.AnglesArray[2];
    STORE_LE_32(&buff[BuffPos], *TempBuff);
    BuffPos += ((uint8_t)4);
    
    TempFloat = TiltSensingMeasure.AnglesArray[0];
    STORE_LE_32(&buff[BuffPos], *TempBuff);
    BuffPos += ((uint8_t)4);
    
    TempFloat = TiltSensingMeasure.AnglesArray[1];
    STORE_LE_32(&buff[BuffPos], *TempBuff);
    
    ret = aci_gatt_srv_write_handle_value_nwk(handle, 0, 2+12,buff);
    if (ret != (tBleStatus)BLE_STATUS_SUCCESS){
      if(BLE_StdErr_Service==BLE_SERV_ENABLE){
        BytesToWrite = (uint8_t)sprintf((char *)BufferToWrite, "Error Updating Tilt Sensing Char\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      } else {
        BLE_MANAGER_PRINTF("Error: Updating Tilt Sensing Char\r\n");
      }
    }
   } else {
    BLE_MANAGER_PRINTF("\r\n\nRead request Tilt Sensing function not defined\r\n\n");
  }

  ret = aci_gatt_srv_authorize_resp_nwk(Connection_Handle, handle,
                                      Operation_Type, 0, Attr_Val_Offset,
                                      Data_Length, Data);
  if( ret != BLE_STATUS_SUCCESS) {
    BLE_MANAGER_PRINTF("aci_gatt_srv_authorize_resp_nwk() failed: 0x%02x\r\n", ret);
  }
}
#endif /* BLUENRG_LP */

