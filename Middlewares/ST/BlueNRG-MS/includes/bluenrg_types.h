/**
  ******************************************************************************
  * @file    bluenrg_types.h
  * @author  AMS - HEA&RF BU
  * @brief   This header file defines the basic data types used by the
  *          BLE stack
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
#ifndef __BLUENRG_TYPES_H__
#define __BLUENRG_TYPES_H__

#include <stdint.h>

#ifndef NULL
#define NULL ((void *)0)
#endif

#ifndef __LITTLE_ENDIAN
#define __LITTLE_ENDIAN 0
#define __BIG_ENDIAN    1
#endif

/* Change this define to 1 if zero-length arrays are not supported by your compiler. */
#define VARIABLE_SIZE   

/* Byte order conversions */
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define htobs(d)  (d)
#define htobl(d)  (d)
#define btohs(d)  (d)
#define btohl(d)  (d)
#elif __BYTE_ORDER == __BIG_ENDIAN
#define htobs(d)  (d<<8|d>>8)
#define htobl(d)  (d<<24|((d<<8)&0x00ff0000)|((d>>8)&0x0000ff00)|((d>>24)&0x000000ff))
#define btohs(d)  (d<<8|d>>8)
#define btohl(d)  (d<<24|((d<<8)&0x00ff0000)|((d>>8)&0x0000ff00)|((d>>24)&0x000000ff))
#else
#error "Unknown byte order"
#endif

typedef uint8_t BOOL;

#ifndef TRUE 
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#ifdef __GNUC__
#undef __packed
#define __packed
#define PACKED __attribute__((packed))
#else
#define PACKED
#endif

#endif /* __BLUENRG_TYPES_H__ */

