/**
  ******************************************************************************
  * @file    glucose_collector_racp.h
  * @author  AMS - VMA, RF Application Team
  * @brief   Glucose collector RACP notifications value header file
  *          Header file for the glucose collector RACP notifications values
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

#ifndef _GLUCOSE_COLLECTOR_RACP_H_
#define _GLUCOSE_COLLECTOR_RACP_H_

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
#include <glucose_collector_racp_CB.h>

/******************************************************************************
* Macro Declarations
******************************************************************************/

/* Max size of a single glucose notification */
#define RACP_SINGLE_NOTIFICATION_MAX_SIZE      (17 + 3) //TBR 3 more bytes for handling up to 3 additional bytes if any)
/******************************************************************************
* Type definitions
******************************************************************************/

/* typedef used for storing the received glucose notifications (measurement, 
   measurement context)for a single RACP procedure  */
typedef struct _tRACPNotifications{
  uint16_t attr_handle; 
  uint8_t data_length; 
  uint8_t data_value[RACP_SINGLE_NOTIFICATION_MAX_SIZE]; 
} tRACPNotifications;
/******************************************************************************
* Imported Variable
******************************************************************************/
/* Storage of received Glucose Sensor  Notifications (measurement, measurement context) 
   for a single RACP procedure */
tRACPNotifications glucose_collector_notifications_for_racp[RACP_MAX_EXPECTED_NOTIFICATIONS_NUMBER];

/******************************************************************************
* Function Declarations
******************************************************************************/

/****** Callbacks ************************************************************/

#endif /* _GLUCOSE_COLLECTOR_RACP_H_ */
