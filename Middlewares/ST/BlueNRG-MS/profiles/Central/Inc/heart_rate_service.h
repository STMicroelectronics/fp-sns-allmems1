/**
  ******************************************************************************
  * @file    heart_rate_collector.h
  * @author  AMS - AAS, RF Application Team
  * @brief   Header file with heart rate service and attribute constant
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2013 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HEART_RATE_SERVICE_H
#define __HEART_RATE_SERVICE_H

/* Defines -------------------------------------------------------------------*/

/***************** SERVICE AND ATTRIBUTE CONSTANT ************************/
/**
 * @brief Max number of primary service supported. 
 * - 1 Generic Attribute (GATT) default service 
 * - 1 Generic Access (GAP) default service 
 * - 1 Heart Rate Service
 * - 1 Device Information Service
 */
#define NUM_MAX_HR_PRIMARY_SERVICE 4  

/**   
 * @brief Max number of Heart Rate characteristics supported for each service
 */
#define NUM_MAX_HR_CHARAC      3 

/**
 * @brief Max number of device information characteristics
 * supported.
 * @note If this value is set to 9 all the devices information
 * are supported
 */
#define NUM_MAX_DEV_INF_CHARAC  9 

/* Heart Rate Measurement Characteristic */
/* Bit Masks for FLAGS Value Fields  */
#define HEART_RATE_VALUE_FORMAT_MASK	        (0x01)  // 00000001 
/** the Sensor Contact Status bits are bit 1 and 2 with value 0,1,2,3 (00, 01, 10, 11)
 *  the bit 2 indicate if the Sensor Contact Feature is supported (10, 11)
 *  the bit 1 indicate if the Sensor Contact Feature is supported and is or not detected 
 */
#define SENSOR_CONTACT_DETECTED_STATUS_MASK     (0x02)  // 00000010 
#define SENSOR_CONTACT_SUPPORTED_STATUS_MASK    (0x04)  // 00000100 
#define ENERGY_EXPENDED_STATUS_MASK             (0x08)  // 00001000
#define RR_INTERVAL_MASK                        (0x10)  // 00010000

/* Keys-Values for BODY SENSOR LOCATION Value Fields  */
#define BODY_SENSOR_LOCATION_OTHER	        (0x00)  
#define BODY_SENSOR_LOCATION_CHEST              (0x01) 
#define BODY_SENSOR_LOCATION_WRIST              (0x02)  
#define BODY_SENSOR_LOCATION_FINGER             (0x03) 
#define BODY_SENSOR_LOCATION_HAND               (0x04) 
#define BODY_SENSOR_LOCATION_EARLOBE            (0x05)  
#define BODY_SENSOR_LOCATION_FOOT               (0x06) 

/* Keys-Values for HEART RATE CONTROL POINT Value Fields  */
#define HR_CNTL_POINT_VALUE_NOT_SUPPORTED       (0x80)
#define HR_CONTROL_POINT_RESET_ENERGY_EXPEND    (0x01)  
#define HR_CONTROL_POINT_RESERVED               (0x00) 
#define HR_CONTROL_POINT_MAX_VALUE              (0xFF) 

/* Procedure timeout (ms) */
//#define HEART_RATE_PROCEDURE_TIMEOUT             30000
/******************************************************************************
* Type definitions
******************************************************************************/

/******************************************************************************
* Function Declarations
******************************************************************************/

#endif /* _HEART_RATE_SERVICE_H_ */
