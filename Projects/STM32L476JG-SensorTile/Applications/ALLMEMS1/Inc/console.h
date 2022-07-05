/**
  ******************************************************************************
  * @file    console.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file provides APIs for demo standard output
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 
#ifndef __CONSOLE_H
#define __CONSOLE_H

#ifdef __cplusplus
 extern "C" {
#endif 
 /* Exported functions ------------------------------------------------------- */
extern int uartSendChar(int ch);
extern int uartReceiveChar(void);
extern int cuiGetInteger(const char* message);

#ifdef __cplusplus
}
#endif

#endif /* __CONSOLE_H */


