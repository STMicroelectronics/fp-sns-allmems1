/**
  ******************************************************************************
  * @file    uuid_ble_service.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   UUID and Macros used for exporting the BLE Characteristics and Services
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
  
/* Define to prevent recursive inclusion -------------------------------------*/  
#ifndef _UUID_BLE_SERVICE_H_
#define _UUID_BLE_SERVICE_H_

#ifdef __cplusplus
 extern "C" {
#endif 

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
{\
  uuid_struct[0 ] = uuid_0 ; uuid_struct[1 ] = uuid_1 ; uuid_struct[2 ] = uuid_2 ; uuid_struct[3 ] = uuid_3 ; \
  uuid_struct[4 ] = uuid_4 ; uuid_struct[5 ] = uuid_5 ; uuid_struct[6 ] = uuid_6 ; uuid_struct[7 ] = uuid_7 ; \
  uuid_struct[8 ] = uuid_8 ; uuid_struct[9 ] = uuid_9 ; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
  uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}

/* Store Value into a buffer in Little Endian Format */
#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

#define STORE_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )

#define STORE_BE_32(buf, val)    ( ((buf)[3] =  (uint8_t) (val)     ) , \
                                   ((buf)[2] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[1] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[0] =  (uint8_t) (val>>24) ) )

#define MCR_FAST_TERM_UPDATE_FOR_OTA(data) aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, 1 , data)

/************************************/
/* Hardware Characteristics Service */
/************************************/
#define COPY_HW_SENS_W2ST_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0xE0,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x00,0x00,0x04,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_MIC_W2ST_CHAR_UUID(uuid_struct)           COPY_UUID_128(uuid_struct,0x04,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_GG_W2ST_CHAR_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0x00,0x02,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/***********************************
* Software Characteristics Service *
*        MotionXX Section          *
************************************/

/* Code for MotionAR integration - Start Section */
#define COPY_ACTIVITY_REC_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x10,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Code for MotionAR integration - End Section */

#ifdef ALLMEMS1_MOTIONCP
#define COPY_CARRY_POSITION_REC_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x08,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#endif /*  ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONFA
#define COPY_FITNESS_ACTIVITIES_W2ST_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x0E,0x00,0x02,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#endif /* ALLMEMS1_MOTIONFA */

/* Code for MotionFX integration - Start Section */
#define COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ECOMPASS_W2ST_CHAR_UUID(uuid_struct)               COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x40,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Code for MotionFX integration - End Section */

/* Code for MotionGR integration - Start Section */
#define COPY_GESTURE_REC_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Code for MotionGR integration - End Section */

/* Code for MotionID integration - Start Section */
#define COPY_INTENSITY_DET_W2ST_CHAR_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x20,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Code for MotionID integration - End Section */

/* Code for MotionTL integration - Start Section */
#define COPY_TILT_MEASUREMENT_W2ST_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x0D,0x00,0x02,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Code for MotionTL integration - End Section */

/* Code for MotionPE & MotionSD & MotionVC integration - Start Section */
#define COPY_MOTION_ALGORITHM_W2ST_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x0A,0x00,0x02,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Code for MotionPE & MotionSD & MotionVC integration - End Section */

/***********************************
* Software Characteristics Service *
*          Audio Section           *
************************************/

/* Code for BlueVoice integration - Start Section */
#define COPY_AUDIO_ADPCM_CHAR_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0x08,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_AUDIO_ADPCM_SYNC_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x40,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
/* Code for BlueVoice integration - End Section */

/******************
* Console Service *
*******************/
#define COPY_CONSOLE_SERVICE_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0E,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TERM_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_STDERR_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/************************
* Configuration Service *
************************/
#define COPY_CONFIG_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0F,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_CONFIG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0F,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
#define COPY_SD_CARD_LOGGING_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x10,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

#ifdef __cplusplus
}
#endif

#endif /* _UUID_BLE_SERVICE_H_ */


