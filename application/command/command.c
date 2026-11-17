/**
 * @file command.c
 * @author anchengc
 * @brief 遥控指令仲裁
 * @version 0.1
 * @date 2026-08-14 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

/* Includes ------------------------------------------------------------------*/
#include "command.h"
#include "math_tools.h"

/* Private variables ---------------------------------------------------------*/
Command_t command;

void Command_Init(void)
{
    command.first_in = 1;
    // Topic 的注册由 robot_sdk 完成

    Create_Publisher(&command.gimbal_cmd_publisher, kCmdGimbalTopicName,
                           sizeof(Gimbal_Cmd_Message_t));
    Create_Publisher(&command.chassis_cmd_publisher, kCmdChassisTopicName,
                           sizeof(Chassis_Cmd_Message_t));
    Create_Publisher(&command.shoot_cmd_publisher, kCmdShootTopicName,
                           sizeof(Shoot_Cmd_Message_t));
    Create_Subscriber(&command.dr16_subscriber, kDr16TopicName,
                            sizeof(command.dr16_msg));
    Create_Subscriber(&command.vt13_subscriber, kVt13TopicName,
                            sizeof(command.vt13_msg));

}

void Command_Update(void)
{
    Message_Read(&command.dr16_subscriber, &command.dr16_msg, sizeof(command.dr16_msg));
    Message_Read(&command.vt13_subscriber, &command.vt13_msg, sizeof(command.vt13_msg));

    uint8_t dr16_online = !Message_IsTimeout(&command.dr16_subscriber, 100);
    uint8_t vt13_online = !Message_IsTimeout(&command.vt13_subscriber, 100);
    Gimbal_Cmd_Message_t gimbal_cmd = {0};
    Chassis_Cmd_Message_t chassis_cmd = {0};
    Shoot_Cmd_Message_t shoot_cmd = {0};
    shoot_cmd.single_shot_seq = command.single_shot_seq;

    if (!dr16_online && !vt13_online)
    {
        command.first_in = 1;
        command.fric_enabled = 0;
        goto publish;
    }

    if (dr16_online)
    {
        if (command.first_in)
        {
            command.last_sw_1 = command.dr16_msg.sw_1;
            command.last_wheel = command.dr16_msg.wheel;
            command.first_in = 0;
        }

        if (command.dr16_msg.sw_2 == 2)
        {
            command.fric_enabled = 0;
            command.first_in = 1;
        }
        else
        {
            // 摩擦轮开关边沿检测
            if (command.dr16_msg.sw_1 == 1 && command.last_sw_1 == 3)
                command.fric_enabled = !command.fric_enabled;
            command.last_sw_1 = command.dr16_msg.sw_1;

            // 单发边沿检测
            if (command.fric_enabled && command.dr16_msg.wheel > 300 &&
                command.last_wheel <= 300)
                command.single_shot_seq++;
            command.last_wheel = command.dr16_msg.wheel;
        }

        switch (command.dr16_msg.sw_2)
        {
        case 2:
            gimbal_cmd.gimbal_mode = GIMBAL_CMD_RELAX;
            chassis_cmd.chassis_mode = CHASSIS_CMD_RELAX;
            break;
        case 3:
            gimbal_cmd.gimbal_mode = GIMBAL_CMD_ACTIVE;
            chassis_cmd.chassis_mode = CHASSIS_CMD_FOLLOW;
            break;
        case 1:
            gimbal_cmd.gimbal_mode = GIMBAL_CMD_ACTIVE;
            chassis_cmd.chassis_mode = CHASSIS_CMD_SPIN;
            break;
        default:
            command.fric_enabled = 0;
            command.first_in = 1;
            break;
        }

        command.auto_aim = command.dr16_msg.wheel < -300;
        gimbal_cmd.auto_aim = command.auto_aim;
        gimbal_cmd.yaw_rate = cubic_map(-command.dr16_msg.ch_2 / 660.0f, 0.5f) * 4.0f;
        gimbal_cmd.pitch_rate = cubic_map(command.dr16_msg.ch_3 / 660.0f, 0.5f) * 5.0f;

        chassis_cmd.rc_vx = cubic_map(command.dr16_msg.ch_1 / 660.0f, 0.5f) * 3.54f;
        chassis_cmd.rc_vy = cubic_map(-command.dr16_msg.ch_0 / 660.0f, 0.5f) * 3.54f;
        chassis_cmd.rc_vw = cubic_map(-command.dr16_msg.ch_2 / 660.0f, 0.5f) * 4.27f;

        shoot_cmd.fric_enabled = command.fric_enabled;
        shoot_cmd.continue_shoot = command.fric_enabled && command.dr16_msg.sw_1 == 2;
        shoot_cmd.single_shot_seq = command.single_shot_seq;
        shoot_cmd.auto_aim = command.auto_aim;
    }
    else if (vt13_online)
    {
        // VT13 控制逻辑尚未实现，保持原来的空分支
    }

publish:
    Message_Publish(&command.gimbal_cmd_publisher, &gimbal_cmd, sizeof(gimbal_cmd));
    Message_Publish(&command.chassis_cmd_publisher, &chassis_cmd, sizeof(chassis_cmd));
    Message_Publish(&command.shoot_cmd_publisher, &shoot_cmd, sizeof(shoot_cmd));
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
