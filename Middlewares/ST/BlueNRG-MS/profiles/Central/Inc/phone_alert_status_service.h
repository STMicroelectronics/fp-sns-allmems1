/**
  ******************************************************************************
  * @file    phone_alert_status_service.h
  * @author  AMS - VMA, RF Application Team
  * @brief   Header file for the phone alert status service and characteristics
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
#ifndef _PHONE_ALERT_STATUS_SERVICE_H_
#define _PHONE_ALERT_STATUS_SERVICE_H_

/******************************************************************************
* Macro Declarations
******************************************************************************/

/******** Phone alert status characteristics *********************************/

/* Alert status (uint8) characteristic value: 3 fields exposing the
   the alerting status of the device */
#define ALERT_STATUS_BIT_RINGER_STATE                 (0x01) /* bit 0 */
#define ALERT_STATUS_BIT_VIBRATOR_STATE               (0x02) /* bit 1 */
#define ALERT_STATUS_BIT_DISPLAY_ALERT_STATUS_STATE   (0x04) /* bit 2 */
/* NOTE: 0x3 - 0x7 bits are reserved for future use */ 

/* Current possible Alert Status combinations values */
#define ALERT_STATUS_RINGER_VIBRATOR_DISPLAY (ALERT_STATUS_BIT_RINGER_STATE | \
                                              ALERT_STATUS_BIT_VIBRATOR_STATE | \
                                              ALERT_STATUS_BIT_DISPLAY_ALERT_STATUS_STATE)
                                             
#define ALERT_STATUS_RINGER_VIBRATOR (ALERT_STATUS_BIT_RINGER_STATE | \
                                      ALERT_STATUS_BIT_VIBRATOR_STATE)
#define ALERT_STATUS_RINGER_DISPLAY (ALERT_STATUS_BIT_RINGER_STATE | \
                                     ALERT_STATUS_BIT_DISPLAY_ALERT_STATUS_STATE)
#define ALERT_STATUS_VIBRATOR_DISPLAY (ALERT_STATUS_BIT_VIBRATOR_STATE | \
                                       ALERT_STATUS_BIT_DISPLAY_ALERT_STATUS_STATE)

#define ALERT_STATUS_NO_ALERTS (0x0) /* no alerts on phone alert status server */

/* Alert status (uint8) characteristic: list of available Key values */ 
#define ALERT_STATUS_RINGER_STATE_NOT_ACTIVE           (0x00)
#define ALERT_STATUS_RINGER_STATE_ACTIVE               (0x01)
#define ALERT_STATUS_VIBRATE_NOT_ACTIVE                (0x00)
#define ALERT_STATUS_VIBRATE_ACTIVE                    (0x01)
#define ALERT_STATUS_DISPLAY_ALERT_STATUS_NOT_ACTIVE   (0x00)
#define ALERT_STATUS_DISPLAY_ALERT_STATUS_ACTIVE       (0x01) 

/* Ringer Setting (8bit) characteristic: list of available Key values */
#define ALERT_STATUS_RINGER_SETTING_SILENT             (0x00)
#define ALERT_STATUS_RINGER_SETTING_NORMAL             (0x01) 
/* NOTE: 0x2, 0xFF keys values are reserved for future use */

/* Ringer Control Point (uint8) characteristic: list of available Key values */
#define ALERT_STATUS_RINGER_CONTROL_POINT_SILENT_MODE        (0x01)
#define ALERT_STATUS_RINGER_CONTROL_POINT_MUTE_ONCE          (0x02)
#define ALERT_STATUS_RINGER_CONTROL_POINT_CANCEL_SILENT_MODE (0x03)
/* NOTE: 0x4 - 0xFF keys values are reserved for future use */

/******************************************************************************
* Type definitions
******************************************************************************/

/******************************************************************************
* Function Declarations
******************************************************************************/

#endif /* _PHONE_ALERT_STATUS_SERVICE_H_ */
