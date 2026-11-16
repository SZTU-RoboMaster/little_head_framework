/**
 * @file command.h
 * @author anchengc
 * @brief 遥控指令仲裁
 * @version 0.1
 * @date 2026-08-14 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "message_center.h"
#include "message_def.h"

typedef struct
{
    uint8_t first_in;
    uint8_t last_sw_1;
    uint8_t fric_enabled;
    uint8_t auto_aim;
    int16_t last_wheel;
    uint32_t single_shot_seq;

    Subscriber_t dr16_subscriber;
    Dr16_Message_t dr16_msg;
    Subscriber_t vt13_subscriber;
    Vt13_Message_t vt13_msg;
    Publisher_t gimbal_cmd_publisher;
    Publisher_t chassis_cmd_publisher;
    Publisher_t shoot_cmd_publisher;
} Command_t;

#ifdef __cplusplus
extern "C" {
#endif

extern Command_t command;
void Command_Init(void);
void Command_Update(void);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
