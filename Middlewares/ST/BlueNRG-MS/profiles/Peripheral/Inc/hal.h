/**
  ******************************************************************************
  * @file    hal.h
  * @author  AMS - HEA&RF BU
  * @brief   Header file which defines Hardware abstraction layer APIs
  *          used by the BLE stack. It defines the set of functions
  *          which needs to be ported to the target platform.
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
#ifndef __HAL_H__
#define __HAL_H__

/******************************************************************************
 * Includes
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/
/* Little Endian buffer to host endianess conversion */
#define LE_TO_HOST_16(ptr)  (uint16_t) ( ((uint16_t) \
                                           *((uint8_t *)ptr)) | \
                                          ((uint16_t) \
                                           *((uint8_t *)ptr + 1) << 8 ) )

#define LE_TO_HOST_32(ptr)   (uint32_t) ( ((uint32_t) \
                                           *((uint8_t *)ptr)) | \
                                           ((uint32_t) \
                                            *((uint8_t *)ptr + 1) << 8)  | \
                                           ((uint32_t) \
                                            *((uint8_t *)ptr + 2) << 16) | \
                                           ((uint32_t) \
                                            *((uint8_t *)ptr + 3) << 24) )

/* Big Endian buffer to host endianess conversion */
#define BE_TO_HOST_16(ptr)  (uint16_t) ( ((uint16_t) \
                                           *((uint8_t *)ptr)) << 8 | \
                                          ((uint16_t) \
                                           *((uint8_t *)ptr + 1) ) )
											
/* Store Value into a buffer in Little Endian Format */
#define HOST_TO_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

#define HOST_TO_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) ) 


/* Store Value into a buffer in Big Endian Format */
#define HOST_TO_BE_16(buf, val)    ( ((buf)[1] =  (uint8_t) (val)    ) , \
                                   ((buf)[0] =  (uint8_t) (val>>8) ) )
 
/******************************************************************************
 * Types
 *****************************************************************************/

/******************************************************************************
 * Function Prototypes
 *****************************************************************************/

#endif /* __HAL_H__ */
