/**
 * @file power_controller.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-08-26 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "message_center.h"
#include "message_def.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    POWER_CONTROL_ENABLE = 0,
    POWER_CONTROL_DISABLE = 1,
} PowerControlMode_e;

/**
 * @brief 底盘功率控制数据
 */
typedef struct
{
    PowerControlMode_e mode;
    float max_power;

    // 订阅者
    Subscriber_t chassis_subscriber;
    Subscriber_t referee_subscriber;

    // 数据
    Chassis_Message_t chassis_msg;
    Referee_Message_t referee_msg;
    Power_Controller_Message_t output_msg;

    // 发布者
    Publisher_t power_controller_publisher;
} PowerController_t;

/* Exported variables ---------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern PowerController_t power_controller;

/* Exported function declarations ---------------------------------------------*/
void PowerController_Init(PowerControlMode_e mode);
void PowerController_Update(void);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
