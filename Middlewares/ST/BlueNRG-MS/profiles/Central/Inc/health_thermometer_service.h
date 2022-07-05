/**
  ******************************************************************************
  * @file    healt_thermometer_service.h
  * @author  AMS - AAS, RF Application Team
  * @brief   Header with health thermometer service and attribute constants
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
#ifndef __HEALTH_THERMOMETER_SERVICE_H
#define __HEALTH_THERMOMETER_SERVICE_H

/* Defines -------------------------------------------------------------------*/

/***************** SERVICE AND ATTRIBUTE CONSTANT ************************/
/**
 * @brief Max number of primary service supported. 
 * - 1 Generic Attribute (GATT) default service 
 * - 1 Generic Access (GAP) default service 
 * - 1 Health Thermometer Service
 * - 1 Device Information Service
 */
#define NUM_MAX_HT_PRIMARY_SERVICE 4  

/**   
 * @brief Max number of Health Thermometer characteristics supported for each service
 */
#define NUM_MAX_HT_CHARAC      4 

/**
 * @brief Max number of device information characteristics
 * supported.
 * @note If this value is set to 9 all the devices information
 * are supported
 */
#define NUM_MAX_DEV_INF_CHARAC  9 

/* TEMPERATURE MEASUREMENT and INTERMEDIATE TEMPERATURE characteristics */
/* Bit Masks for FLAGS Value Fields  */
/* Temperature Unit in Celsius (bit 0 with value 0), in Fahrenheit (bit 0 with value 1) */
#define FLAG_TEMPERATURE_UNITS_FARENHEIT 	  (0x01)
#define FLAG_TIMESTAMP_PRESENT 		          (0x02)
#define FLAG_TEMPERATURE_TYPE 		          (0x04)

/* Keys-Values for TEMPERATURE TYPE Value Fields  */
#define TEMPERATURE_TYPE_ARMPIT                    (0x01) 
#define TEMPERATURE_TYPE_BODY                      (0x02)  
#define TEMPERATURE_TYPE_EAR                       (0x03) 
#define TEMPERATURE_TYPE_FINGER                    (0x04) 
#define TEMPERATURE_TYPE_GASTRO_INTESTINAL_TRACT   (0x05)  
#define TEMPERATURE_TYPE_MOUTH                     (0x06) 
#define TEMPERATURE_TYPE_RECTUM                    (0x07) 
#define TEMPERATURE_TYPE_TOE                       (0x08)  
#define TEMPERATURE_TYPE_TYMPANUM                  (0x09) 

/* Keys-Values for MEASUREMENT INTERVAL Value Fields */
#define OUT_OF_RANGE                      (0xFF)
#define MEASUREMENT_INTERVAL_MIN_VALUE    (0x0001)  
#define MEASUREMENT_INTERVAL_MAX_VALUE    (0xFFFF) //max value 65535

#define UNIT_TEMP_CELSIUS       0x272F
#define UNIT_TEMP_FAHRENHEIT    0x27AC    
                  
/******************************************************************************
* Type definitions
******************************************************************************/

/******************************************************************************
* Function Declarations
******************************************************************************/

#endif /* _HEALTH_THERMOMETER_SERVICE_H_ */
