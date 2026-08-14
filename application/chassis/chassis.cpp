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

#include "chassis.h"

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
void Chassis::init()
{

    // 底盘角速度PID
    omega_pid_.init(5.0f, 0.0f, 0.0f);

    // 轮向电机初始化
    for (int i = 0; i < 4; i++)
    {
        wheel_motor_[i].omega_pid_.init(400.0f, 0.0f, 0.0f);
    }

    wheel_motor_[0].init(&hcan1, 0x200, 0x201, MOTOR_DJI_CONTROL_METHOD_OMEGA, (3591.0f / 187.0f));
    wheel_motor_[1].init(&hcan1, 0x200, 0x202, MOTOR_DJI_CONTROL_METHOD_OMEGA, (3591.0f / 187.0f));
    wheel_motor_[2].init(&hcan1, 0x200, 0x203, MOTOR_DJI_CONTROL_METHOD_OMEGA, (3591.0f / 187.0f));
    wheel_motor_[3].init(&hcan1, 0x200, 0x204, MOTOR_DJI_CONTROL_METHOD_OMEGA, (3591.0f / 187.0f));

    gimbal_subscriber_ = MessageCenter::instance().subscribe<GimbalMessage>(kGimbalTopicName);
    cmd_subscriber_ = MessageCenter::instance().subscribe<ChassisCmdMessage>(kCmdChassisTopicName);
}

void Chassis::update_input()
{
    gimbal_subscriber_.update(gimbal_msg_);
    cmd_subscriber_.update(cmd_msg_);
}

void Chassis::update_feedback()
{
    for (int i = 0; i < 4; i++)
    {
        feedback_.wheel_omega[i] = wheel_motor_[i].rx_data_.omega;
    }
    mecanum_forward_kinematics();
    feedback_.gimbal_yaw = gimbal_msg_.yaw_total_angle;
}

void Chassis::handle_safety()
{
    // 安全处理
    if (!cmd_subscriber_.is_fresh(100))
    {
        cmd_msg_ = {};
    }
}

void Chassis::set_mode()
{
    // 设置模式
    switch (cmd_msg_.chassis_mode)
    {
    case CHASSIS_CMD_RELAX:
        mode_ = CHASSIS_RELAX;
        break;
    case CHASSIS_CMD_FOLLOW:
        mode_ = CHASSIS_FOLLOW;
        break;
    case CHASSIS_CMD_SPIN:
        mode_ = CHASSIS_SPIN;
        break;
    default:
        break;
    }
}

void Chassis::control()
{
    // 控制
    float yaw_error = 0.0f;
    switch (mode_)
    {
    case CHASSIS_RELAX:
    {
        for (int i = 0; i < 4; i++)
        {
            wheel_motor_[i].set_control_method(MOTOR_DJI_CONTROL_METHOD_CURRENT);
            control_output_.wheel_target_current[i] = 0.0f;
        }
        break;
    }
    case CHASSIS_ONLY:
    {
        for (int i = 0; i < 4; i++)
        {
            wheel_motor_[i].set_control_method(MOTOR_DJI_CONTROL_METHOD_OMEGA);
        }
        control_output_.target_velocity_x = cmd_msg_.rc_vx;
        control_output_.target_velocity_y = cmd_msg_.rc_vy;
        control_output_.target_omega = cmd_msg_.rc_vw;
        break;
    }
    case CHASSIS_FOLLOW:
    {
        yaw_error = wrap_center((feedback_.gimbal_yaw - config_.gimbal_yaw_offset), (2.0f * PI));
        omega_pid_.set_target(0.0f);
        omega_pid_.set_feedback(yaw_error);
        omega_pid_.calculate();
        for (int i = 0; i < 4; i++)
        {
            wheel_motor_[i].set_control_method(MOTOR_DJI_CONTROL_METHOD_OMEGA);
        }
        control_output_.target_velocity_x = cmd_msg_.rc_vx;
        control_output_.target_velocity_y = cmd_msg_.rc_vy;
        control_output_.target_omega = -omega_pid_.get_output();
        break;
    }
    case CHASSIS_SPIN:
    {
        for (int i = 0; i < 4; i++)
        {
            wheel_motor_[i].set_control_method(MOTOR_DJI_CONTROL_METHOD_OMEGA);
        }

        yaw_error = wrap_center((feedback_.gimbal_yaw - config_.gimbal_yaw_offset), (2.0f * PI));
        float sin_yaw = arm_sin_f32(yaw_error);
        float cos_yaw = arm_cos_f32(yaw_error);
        float vx_temp = cmd_msg_.rc_vx;
        float vy_temp = cmd_msg_.rc_vy;
        control_output_.target_velocity_x = vx_temp * cos_yaw + vy_temp * sin_yaw;
        control_output_.target_velocity_y = vx_temp * (-sin_yaw) + vy_temp * cos_yaw;
        control_output_.target_omega = config_.spin_vw;
        break;
    }
    }
}

void Chassis::solve()
{
    if (mode_ == CHASSIS_RELAX)
    {
        return;
    }
    else
    {
        // 解算
        mecanum_inverse_kinematics();
    }
}

void Chassis::output()
{
    // 输出
    if (mode_ == CHASSIS_RELAX)
    {
        for (int i = 0; i < 4; i++)
        {
            wheel_motor_[i].set_target_current(control_output_.wheel_target_current[i]);
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            wheel_motor_[i].set_target_omega(control_output_.wheel_target_omega[i]);
        }
    }
}

void Chassis::mecanum_forward_kinematics()
{
    // 麦轮解算
    feedback_.velocity_x = (-feedback_.wheel_omega[0] + feedback_.wheel_omega[1] +
                            feedback_.wheel_omega[2] - feedback_.wheel_omega[3]) *
                           config_.wheel_radius / 4.0f;
    feedback_.velocity_y = (-feedback_.wheel_omega[0] - feedback_.wheel_omega[1] +
                            feedback_.wheel_omega[2] + feedback_.wheel_omega[3]) *
                           config_.wheel_radius / 4.0f;
    feedback_.omega = (-feedback_.wheel_omega[0] - feedback_.wheel_omega[1] -
                       feedback_.wheel_omega[2] - feedback_.wheel_omega[3]) *
                      config_.wheel_radius / 4.0f;
}

void Chassis::mecanum_inverse_kinematics()
{
    // 麦轮逆解算
    control_output_.wheel_target_omega[0] =
        (-control_output_.target_velocity_x - control_output_.target_velocity_y -
         control_output_.target_omega * config_.wheel_to_center_distance) /
        config_.wheel_radius;
    control_output_.wheel_target_omega[1] =
        (control_output_.target_velocity_x - control_output_.target_velocity_y -
         control_output_.target_omega * config_.wheel_to_center_distance) /
        config_.wheel_radius;
    control_output_.wheel_target_omega[2] =
        (control_output_.target_velocity_x + control_output_.target_velocity_y -
         control_output_.target_omega * config_.wheel_to_center_distance) /
        config_.wheel_radius;
    control_output_.wheel_target_omega[3] =
        (-control_output_.target_velocity_x + control_output_.target_velocity_y -
         control_output_.target_omega * config_.wheel_to_center_distance) /
        config_.wheel_radius;
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
