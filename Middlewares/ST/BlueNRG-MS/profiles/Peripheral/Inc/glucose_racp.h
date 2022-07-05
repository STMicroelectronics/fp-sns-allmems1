/**
  ******************************************************************************
  * @file    glucose_racp.h
  * @author  AMS - VMA, RF Application Team
  * @brief   Header file for the glucose record access point handler
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

/******************************************************************************
 * Include Files
******************************************************************************/

#ifndef _GLUCOSE_RACP_H_
#define _GLUCOSE_RACP_H_

#include <host_config.h>

#include <glucose_sensor.h>

/******************************************************************************
* Macro Declarations
******************************************************************************/

/******************************************************************************
* Type definitions
******************************************************************************/


/******************************************************************************
* Imported Variable
******************************************************************************/

/******************************************************************************
* Function Declarations
******************************************************************************/
/**
  * @brief  Reset all the not deleted records to  non filtered status
  *         before doing a new operation.
  * @param  [in] action : TRUE (reset flag); FALSE (set flag)
  * @retval number of not deleted records (TBR)
  */
uint16_t glucoseDatabase_ResetAllNotDeletedRecords(BOOL action);

/**
  * @brief  Count number of not delelted records on database
  * @retval number of not deleted records (TBR)
  */
uint16_t  glucoseDatabase_NumNotDeletedRecords(void);

/**
  * @brief  Find next filtered record on glucose database
  * @param  [in] database_index : index from which starting the search
  * @retval database index
  */
uint16_t glucoseDataBase_FindNextSelectedRecord(uint16_t database_index);

/**
  * @brief RACP request handler 
  * @param [in] attValueLen : received RACP request lenght
  * @retval none
  */
void glucoseRACP_RequestHandler(uint8_t attValueLen);

/**
  * @brief Manage the notification of next glucose measurment from database records 
  *        It should be called periodically on notification timer timeout
  * @retval TRUE : if other selected measurement has to be sent; false otherwise
  */
BOOL glucoseRACP_SendNextMeasurementRecord(void);

/* UTILITIES */

/**
  * @brief  Change current database sequence numbers and restore status to not deleted
  *         to simulate new measurements on user initiative
  *         (only for internal test purposes) (TBR).
  * @param  [in] num_record : number of records to be restored
  * @param  [in] flag_status : flag status
  * @retval none
  */ 
void glucoseDatabase_Change_Database_Manually(uint16_t num_record, uint8_t flag_status);

#endif /* _GLUCOSE_RACP_H_ */
