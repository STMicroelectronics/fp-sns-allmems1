/**
  ******************************************************************************
  * @file    hid_device_i.c
  * @author  AMS - HEA&RF BU
  * @brief   Internal header file for HID profile
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2012 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef _HID_DEVICE_I_H_
#define _HID_DEVICE_I_H_

/*******************************************************************************
* Macros
*******************************************************************************/

/* HID profile main states */
#define HID_DEVICE_STATE_UNINITIALIZED                 (0x00)
#define HID_DEVICE_STATE_WAITING_BLE_INIT              (0x10)

#define HID_DEVICE_STATE_INITIALIZED                   (0x80)
#define HID_DEVICE_STATE_ADVERTIZE                     (0x90)
#define HID_DEVICE_STATE_CONNECTED                     (0xB0)
#define HID_DEVICE_STATE_INVALID                       (0xFF)

/* substates for HID_DEVICE_STATE_ADVERTIZE */
#define HID_DEVICE_STATE_ADVERTISABLE_IDLE                            (0x91)
#define HID_DEVICE_STATE_ADVERTISING_FOR_WHITE_LIST_DO_START          (0x92)
#define HID_DEVICE_STATE_ADVERTISING_FOR_WHITE_LIST_STARTING          (0x93)
#define HID_DEVICE_STATE_ADVERTISING_FOR_WHITE_LIST                   (0x94)
#define HID_DEVICE_STATE_ADVERTISING_FOR_WHITE_LIST_TIMEOUT_DO_STOP   (0x95)

#define HID_DEVICE_STATE_FAST_DISCOVERABLE_DO_START                   (0x97)
#define HID_DEVICE_STATE_FAST_DISCOVERABLE_STARTING                   (0x98)
#define HID_DEVICE_STATE_FAST_DISCOVERABLE                            (0x99)
#define HID_DEVICE_STATE_FAST_DISCOVERABLE_TIMEOUT_DO_STOP            (0x9A)
#define HID_DEVICE_STATE_FAST_DISCOVERABLE_STOPPING                   (0x9B)
#define HID_DEVICE_STATE_LOW_POWER_DISCOVERABLE_DO_START              (0x9C)
#define HID_DEVICE_STATE_LOW_POWER_DISCOVERABLE_STARTING              (0x9D)
#define HID_DEVICE_STATE_LOW_POWER_DISCOVERABLE                       (0x9E)
#define HID_DEVICE_STATE_LOW_POWER_DISCOVERABLE_TIMEOUT_DO_STOP       (0x9F)

#define HID_DEVICE_STATE_BLE_ADVERTISING                              (0xA1)
#define HID_DEVICE_STATE_PROCESSING_PAIRING                           (0xA2)

/* substates under HID_DEVICE_STATE_CONNECTED main state */
#define HID_DEVICE_STATE_CONNECTED_IDLE                               (0xB1)
#define HID_DEVICE_STATE_CONNECTED_BUSY                               (0xB2)

#define HID_DEVICE_STATE_DISCONNECTING                                (0xF2)
#define HID_DEVICE_STATE_DISCONNECTED                                 (0xF3)

#define HCI_CMD_NO_OPERATION			       (0x0000)
#define HCI_VENDOR_CMD_GATT_UPD_CHAR_VAL               (0xFD06)

/******************************************************************************
 * Function Prototypes
******************************************************************************/
void HIDProfile_Write_MainStateMachine(tProfileState localmainState);
tProfileState HIDProfile_Read_MainStateMachine(void);
void HIDProfile_Write_SubStateMachine(tProfileState localsubState);
tProfileState HIDProfile_Read_SubStateMachine(void);

/******************************************************************************
 * type definitions
******************************************************************************/
/**
 * HID profile context
 */ 
typedef struct _tHidDeviceContext
{
  /**
   * main state of the HID
   * profile state machine
   */ 
  tProfileState mainState;
	
  /**
   * sub state of the HID
   * profile state machine
   */ 
  tProfileState subState;
	
  /**
   * will contain the timer ID of the 
   * timer started by the HID profile
   */ 
  tTimerID timerID;
   
  /**
   * connection handle
   */ 
  uint16_t connHandle;
   
  /**
   * callback to notify the applciation with
   * the profile specific events
   */ 
  BLE_CALLBACK_FUNCTION_TYPE applicationNotifyFunc;

  /**
   * number of HID services exposed
   * by the profile
   */ 
  uint8_t numofHIDServices;
	
  /**
   * HID service currently under process
   */ 
  uint8_t currHidServIndx;
	
  /**
   * HID service parameters for each of the
   * HID services supported
   */ 
  tHidServiceContext hidService[HID_SERVICE_MAX];

  /**
   * number of battery services
   */ 
  uint8_t numofBatteryServices;
	
  /**
   * Battery service currently under process
   */ 
  uint8_t currBatteryServIndx;
	
  /**
   * Battery service parameters for each of the
   * battery services supported
   */ 
  tBatteryServiceContext batteryService[BATTERY_SERVICE_MAX];
    
  /**
   * scan parameters service parameters
   */ 
  tScanParamService scanParamService; 
    
  /**
   * interface structure used to register
   * the HID profile with the main profile
   */ 
  tProfileInterfaceContext HIDtoBLEInf;
  /**
   * BLE Procedure ongoing
   */
  uint16_t opCode;
  
  /**
   * service handle of current notified characteristic
   */ 
  uint16_t updater_service_handle;
	
  /**
   * characteristic handle of current notified characteristic
   */ 
  uint16_t updater_char_handle;
    
}tHidDeviceContext;



#endif /* _HID_DEVICE_I_H_ */
