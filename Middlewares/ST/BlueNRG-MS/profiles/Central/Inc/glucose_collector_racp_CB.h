/**
  ******************************************************************************
  * @file    glucose_collector_racp_CB.h
  * @author  AMS - VMA, RF Application Team
  * @brief   Glucose collector racp callbacks header file
  *          Header file for the glucose collector racp callbacks
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/******************************************************************************
 * Include Files
******************************************************************************/

#ifndef _GLUCOSE_COLLECTOR_RACP_CB_H_
#define _GLUCOSE_COLLECTOR_RACP_CB_H_

#include <host_config.h>
#include <hci.h>

#include "bluenrg_aci.h"
#include "bluenrg_gatt_server.h"
#include "hci_const.h"
#include "bluenrg_gap.h"
#include "sm.h"

#include <debug.h>
#include <ble_list.h>
#include <timer.h>
#include <uuid.h>

/******************************************************************************
* Macro Declarations
******************************************************************************/
/* Max number of expected notifications recevied as consequence of a 
   single RACP procedure: To BE TUNED based on expected glucose 
   sensor max notifications*/ 
#define RACP_MAX_EXPECTED_NOTIFICATIONS_NUMBER 255
/******************************************************************************
* Type definitions
******************************************************************************/

/******************************************************************************
* Imported Variable
******************************************************************************/

/******************************************************************************
* Function Declarations
******************************************************************************/

/****** Callbacks ************************************************************/

/**
* @brief  It stores the Glucose Sensor  Notifications (measurement, measurement context) 
*         for a single RACP procedure 
* @param  attr_handle: glucose sensor characteristic handle
* @param  data_lenght: glucose sensor characteristic value lenght
* @param  value: glucose sensor characteristic value 
*/
void GL_Collector_RACP_Notifications_Storage(uint16_t attr_handle, uint8_t data_length,uint8_t * value);

/**
* @brief  Utility function: It performs a post processing of the received notifications 
* to a single RACP procedure. It is used on PTS validation for display notifications data values
* @param  None
*/
void GL_Collector_RACP_Notifications_PostProcessing(void);

#endif /* _GLUCOSE_COLLECTOR_RACP_CB_H_ */
