/**
  ******************************************************************************
  * @file    timer.h
  * @author  AMS - VMA, RF Application Team
  * @brief   Header file for timer.c
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
#ifndef _TIMER_H_
#define _TIMER_H_

#include <ble_list.h>
#include "bluenrg_def.h"

/******************************************************************************
 * Macro Declarations
******************************************************************************/
#define MAX_TIMER_ENTRIES	(2)

/******************************************************************************
*  Type Declarations
******************************************************************************/
typedef void (* TIMER_TIMEOUT_NOTIFY_CALLBACK_TYPE)(void);
typedef uint8_t tTimerID;

typedef struct _tTimerData
{
	tListNode this_node;
	long int expiryTime;
	TIMER_TIMEOUT_NOTIFY_CALLBACK_TYPE timercb;
	tTimerID timerID;
}tTimerData;

typedef struct _tTimerContext
{	
    /**
     * Timer Q - Link list of nodes of tTimerData that have to be processed
     */ 
    tListNode timerQ;
	
	/**
	 * timerPool - linked list of the free timer nodes
	 */ 
	tListNode timerPool;
	
	/**
	 * gives the number of entries in the 
	 * timerQ
	 */
     uint8_t timerNeeded;
 
	/**
	 * contains the expiry time of the currently running
	 * timer
	 */ 
	long int expiryTime;
	
	/**
	 * memory for timer nodes
	 */ 
	tTimerData timerData[MAX_TIMER_ENTRIES];
}tTimerContext; 

/******************************************************************************
*  Function Prototypes
******************************************************************************/
tBleStatus Blue_NRG_Timer_Init(void);

tBleStatus Blue_NRG_Timer_Start(uint32_t expiryTime,
                                TIMER_TIMEOUT_NOTIFY_CALLBACK_TYPE timercb,
                                tTimerID *timerID);

tBleStatus Blue_NRG_Timer_Stop(tTimerID timerID);

void Blue_NRG_Timer_Process_Q(void);

void Blue_NRG_Timer_ExpiryCallback(void);

#endif /* _TIMER_H_ */
