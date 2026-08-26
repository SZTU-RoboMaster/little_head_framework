/**
 * @file template.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "command.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief
 *
 * @param

 */
void Command::init()
{
    gimbal_cmd_publisher_ =
        MessageCenter::instance().advertise<GimbalCmdMessage>(kCmdGimbalTopicName);
    chassis_cmd_publisher_ =
        MessageCenter::instance().advertise<ChassisCmdMessage>(kCmdChassisTopicName);
    shoot_cmd_publisher_ =
        MessageCenter::instance().advertise<ShootCmdMessage>(kCmdShootTopicName);

    dr16_subscriber_ = MessageCenter::instance().subscribe<Dr16Message>(kDr16TopicName);
    vt13_subscriber_ = MessageCenter::instance().subscribe<Vt13Message>(kVt13TopicName);
}

void Command::update()
{
    // 更新遥控器数据
    dr16_subscriber_.update(dr16_msg_);
    vt13_subscriber_.update(vt13_msg_);

    // 检查在线状态
    uint8_t dr16_online = dr16_subscriber_.is_fresh(100);
    uint8_t vt13_online = vt13_subscriber_.is_fresh(100);

    GimbalCmdMessage gimbal_cmd = {};
    ChassisCmdMessage chassis_cmd = {};
    ShootCmdMessage shoot_cmd = {};
    shoot_cmd.single_shot_seq = single_shot_seq_;

    // 离线处理
    if (!dr16_online && !vt13_online)
    {
        first_in_ = true;
        fric_enabled_ = false;
        gimbal_cmd_publisher_.publish(gimbal_cmd);
        chassis_cmd_publisher_.publish(chassis_cmd);
        shoot_cmd_publisher_.publish(shoot_cmd);
        return;
    }

    // 数据仲裁
    if (dr16_online)
    {
        if (first_in_)
        {
            last_sw_1_ = dr16_msg_.sw_1;
            last_wheel_ = dr16_msg_.wheel;
            first_in_ = false;
        }

        // 失能
        if (dr16_msg_.sw_2 == 2)
        {
            fric_enabled_ = false;
            first_in_ = true;
        }
        // 使能
        else
        {
            // 摩擦轮开关边沿检测
            if (dr16_msg_.sw_1 == 1 && last_sw_1_ == 3)
            {
                fric_enabled_ = !fric_enabled_;
            }
            last_sw_1_ = dr16_msg_.sw_1;

            // 单发边沿检测
            if (fric_enabled_ && dr16_msg_.wheel > 300 && last_wheel_ <= 300)
            {
                single_shot_seq_++;
            }
            last_wheel_ = dr16_msg_.wheel;
        }

        switch (dr16_msg_.sw_2)
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
            gimbal_cmd.gimbal_mode = GIMBAL_CMD_RELAX;
            chassis_cmd.chassis_mode = CHASSIS_CMD_RELAX;
            fric_enabled_ = false;
            first_in_ = true;
            break;
        }
        gimbal_cmd.yaw_rate = cubic_map(-dr16_msg_.ch_2 / 660.0f, 0.5f) * 4.0f;
        gimbal_cmd.pitch_rate = cubic_map(dr16_msg_.ch_3 / 660.0f, 0.5f) * 5.0f;

        chassis_cmd.rc_vx = cubic_map(dr16_msg_.ch_1 / 660.0f, 0.5f) * 3.54f;
        chassis_cmd.rc_vy = cubic_map(-dr16_msg_.ch_0 / 660.0f, 0.5f) * 3.54f;
        chassis_cmd.rc_vw = cubic_map(-dr16_msg_.ch_2 / 660.0f, 0.5f) * 4.27f;

        shoot_cmd.fric_enabled = fric_enabled_;
        shoot_cmd.continue_shoot = fric_enabled_ && (dr16_msg_.sw_1 == 2);
        shoot_cmd.single_shot_seq = single_shot_seq_;
    }
    else if (vt13_online)
    {
    }

    gimbal_cmd_publisher_.publish(gimbal_cmd);
    chassis_cmd_publisher_.publish(chassis_cmd);
    shoot_cmd_publisher_.publish(shoot_cmd);
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
