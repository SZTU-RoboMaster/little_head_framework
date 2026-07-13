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
#include "gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Chassis chassis;

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
        wheel_motor_[i].omega_pid_.init(300.0f, 0.0f, 0.0f);
    }

    wheel_motor_[0].init(&hcan1, 0x200, 0x201, MOTOR_DJI_CONTROL_METHOD_OMEGA, (3591.0f / 187.0f));
    wheel_motor_[1].init(&hcan1, 0x200, 0x202, MOTOR_DJI_CONTROL_METHOD_OMEGA, (3591.0f / 187.0f));
    wheel_motor_[2].init(&hcan1, 0x200, 0x203, MOTOR_DJI_CONTROL_METHOD_OMEGA, (3591.0f / 187.0f));
    wheel_motor_[3].init(&hcan1, 0x200, 0x204, MOTOR_DJI_CONTROL_METHOD_OMEGA, (3591.0f / 187.0f));
}

void Chassis::update_input()
{
    // 读取遥控器输入
    input_.ch_1 = dr16_->data_.ch_1;
    input_.ch_0 = -dr16_->data_.ch_0;
    input_.ch_2 = -dr16_->data_.ch_2;
    input_.sw_2 = dr16_->data_.sw_2;
    input_.gimbal_yaw = gimbal.motor_yaw_.rx_data_.total_angle;
}

void Chassis::update_feedback()
{
    for (int i = 0; i < 4; i++)
    {
        feedback_.wheel_omega[i] = wheel_motor_[i].rx_data_.omega;
    }
    mecanum_forward_kinematics();
}

void Chassis::handle_safety()
{
    // 安全处理
}

void Chassis::set_mode()
{
    // 设置模式
    switch (input_.sw_2)
    {
    case 2:
        mode_ = CHASSIS_RELAX;
        break;
    case 3:
        mode_ = CHASSIS_ONLY;
        break;
    case 1:
        mode_ = CHASSIS_FOLLOW;
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
        control_output_.target_velocity_x = cubic_map(input_.ch_1 / 660.0f, 0.5f) * 4.27f;
        control_output_.target_velocity_y = cubic_map(input_.ch_0 / 660.0f, 0.5f) * 4.27f;
        control_output_.target_omega = cubic_map(input_.ch_2 / 660.0f, 0.5f) * 4.27f;
        break;
    }
    case CHASSIS_FOLLOW:
    {
        yaw_error = wrap_center((input_.gimbal_yaw - config_.gimbal_yaw_offset), (2.0f * PI));
        omega_pid_.set_target(0.0f);
        omega_pid_.set_feedback(yaw_error);
        omega_pid_.calculate();
        for (int i = 0; i < 4; i++)
        {
            wheel_motor_[i].set_control_method(MOTOR_DJI_CONTROL_METHOD_OMEGA);
        }
        control_output_.target_velocity_x = input_.ch_1 / 660.0f * 4.27f;
        control_output_.target_velocity_y = input_.ch_0 / 660.0f * 4.27f;
        control_output_.target_omega = -omega_pid_.get_output();
        break;
    }
    case CHASSIS_SPIN:
    {
        // for (int i = 0; i < 4; i++)
        // {
        //     wheel_motor_[i].set_control_method(MOTOR_DJI_CONTROL_METHOD_OMEGA);
        // }
        // control_output_.target_velocity_x = 0.0f;
        // control_output_.target_velocity_y = 0.0f;
        // control_output_.target_omega = input_.ch_1 * 0.0f;
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
