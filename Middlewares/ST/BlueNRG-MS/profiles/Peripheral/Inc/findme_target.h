/**
  ******************************************************************************
  * @file    findme_target.h
  * @author  AMS - VMA, RF Application Team
  * @brief   Header for the find me target profile
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2014 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef _FIND_ME_TARGET_
#define _FIND_ME_TARGET_

/**
 * FindMeTarget_Init
 * 
 * @param[in] FindMeTargetcb : callback function to be called
 *           by the profile to notify the application of events
 * 
 * Initializes the Find Me Target profile
 */ 
tBleStatus FindMeTarget_Init(BLE_CALLBACK_FUNCTION_TYPE FindMeTargetcb);
							 
/**
 * FMT_Advertize
 * 
 * This function puts the device into
 * discoverable mode if it is in the
 * proper state to do so
 */ 
tBleStatus FMT_Advertize(void);

/**
 * FMT_Add_Device_To_WhiteList
 * 
 * @param[in] addrType : address type of the bdAddr
 *            to be added to the whitelist
 * @param[in] bdAddr : address of the peer device
 *            that has to be added to the whitelist
 * 
 * Adds a device to the whitelist
 */ 
tBleStatus FMT_Add_Device_To_WhiteList(uint8_t addrType,uint8_t* bdAddr);

/**
 * FMT_LimDisc_Timeout_Handler
 * 
 * This function is called when the advertising
 * for fast connection times out
 */ 
void FMT_LimDisc_Timeout_Handler(void);

/**
 * FMTProfile_StateMachine
 * 
 * @param[in] None
 * 
 * FMT profile's state machine: to be called on application main loop. 
 */ 
tBleStatus FMTProfile_StateMachine(void);

#endif /* _FIND_ME_TARGET_ */
