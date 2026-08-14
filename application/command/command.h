/**
 * @file template.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "math_tools.h"

#include "message_center.h"
#include "message_def.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief Specialized
 *
 */
class Command
{
public:
    void init();

    void update();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量
    uint8_t first_in_ = true;

    uint8_t last_sw_1_;
    uint8_t fric_enabled_ = false;
    int16_t last_wheel_;
    uint32_t single_shot_seq_ = 0;

    // 读变量
    Subscriber<Dr16Message> dr16_subscriber_;
    Dr16Message dr16_msg_;
    Subscriber<Vt13Message> vt13_subscriber_;
    Vt13Message vt13_msg_;

    // 写变量
    Publisher<GimbalCmdMessage> gimbal_cmd_publisher_;
    Publisher<ChassisCmdMessage> chassis_cmd_publisher_;
    Publisher<ShootCmdMessage> shoot_cmd_publisher_;

    // 读写变量

    // 内部函数
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
