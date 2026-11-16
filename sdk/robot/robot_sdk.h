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

/* Exported macros -----------------------------------------------------------*/
#define GIMBAL_BOARD
#define CHASSIS_BOARD
/* Exported types ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

    void robot_sdk_message_init(void);
    void robot_sdk_init();

#ifdef __cplusplus
}
#endif

void robot_sdk_start();

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
