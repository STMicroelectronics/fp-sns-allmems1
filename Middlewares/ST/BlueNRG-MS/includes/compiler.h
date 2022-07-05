/**
  ******************************************************************************
  * @file    compiler.h
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
#ifndef COMPILER_H
#define COMPILER_H

#ifdef __GNUC__
#undef __packed
#define __packed
#define PACKED __attribute__((packed))
#else
#define PACKED
#endif /* __GNUC__ */

#endif /* COMPILER_H */
