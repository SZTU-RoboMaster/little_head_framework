/**
 * @file robot_sdk.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-05 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "message_center.h"
/* Exported macros -----------------------------------------------------------*/
//#define GIMBAL_BOARD
#define CHASSIS_BOARD

/* Exported types ------------------------------------------------------------*/
extern Subscriber_t boardcomm_chassiscmd_subscriber;

extern Publisher_t boardcomm_referee_publisher;

extern Publisher_t boardcomm_chassiscmd_publisher;

extern Subscriber_t boardcomm_referee_subscriber;
/* Exported variables ---------------------------------------------------------*/


/* Exported function declarations ---------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

    void RobotSdk_Init(void);

    void RobotSdk_Start(void);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
