/**
  ******************************************************************************
  * @file    host_config.h
  * @author  AMS - VMA, RF Application Team
  * @brief   Defines the host stack configuration values
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
#ifndef _HOST_CONFIG_H_
#define _HOST_CONFIG_H_

#define PLATFORM_LINUX         (0x01)
#define PLATFORM_WINDOWS       (0x02)
#define PLATFORM_ARM           (0x03)

#define PLATFORM_TYPE          (PLATFORM_ARM)

/* profile codes */
#define HEART_RATE_COLLECTOR          (0x01)
#define PROXIMITY_REPORTER            (0x02)   // Not supported yet
#define PROXIMITY_MONITOR             (0x04)   // Not supported yet
#define TIME_SERVER                   (0x08)   // Not supported yet
#define TIME_CLIENT                   (0x10)
#define HID_HOST                      (0x20)   // Not supported yet
#define FIND_ME_TARGET                (0x40)   // Not supported yet
#define FIND_ME_LOCATOR               (0x80)
#define PHONE_ALERT_STATUS_SERVER     (0x100)  // Not supported yet
#define BLOOD_PRESSURE_COLLECTOR      (0x200)
#define HEALTH_THERMOMETER_COLLECTOR  (0x400)
#define ALERT_NOTIFICATION_CLIENT     (0x800)
#define ALERT_NOTIFICATION_SERVER     (0x1000) // Not supported yet
#define GLUCOSE_COLLECTOR             (0x2000) // Not supported by L0

/* It is only used for building the Profiles library */
/*
//#define ALL_PROFILES			(HEART_RATE_COLLECTOR|\
//                                         PROXIMITY_REPORTER|\
//                                         PROXIMITY_MONITOR|\
//                                         TIME_SERVER|\
//                                         TIME_CLIENT|\
//                                         FIND_ME_TARGET|\
//                                         FIND_ME_LOCATOR|\
//                                         PHONE_ALERT_STATUS_SERVER|\
//                                         BLOOD_PRESSURE_COLLECTOR|\
//                                         HEALTH_THERMOMETER_COLLECTOR|\
//                                         ALERT_NOTIFICATION_CLIENT|\
//                                         ALERT_NOTIFICATION_SERVER|\
//                                         GLUCOSE_COLLECTOR|\
//                                         HID_HOST)
*/

#define ALL_PROFILES (HEART_RATE_COLLECTOR|\
                      TIME_CLIENT|\
                      FIND_ME_LOCATOR|\
                      BLOOD_PRESSURE_COLLECTOR|\
                      HEALTH_THERMOMETER_COLLECTOR|\
                      GLUCOSE_COLLECTOR|\
                      ALERT_NOTIFICATION_CLIENT)

/* active profiles */
#ifndef BLE_CURRENT_PROFILE_ROLES
#define BLE_CURRENT_PROFILE_ROLES  HEART_RATE_COLLECTOR
#endif

#ifdef  __ECLIPSE_BASED_IDE__
#define __weak __attribute__((weak))
#endif

#endif /* _HOST_CONFIG_H_ */ 
