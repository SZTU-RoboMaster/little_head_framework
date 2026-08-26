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
        wheel_motor_[i].omega_pid_.init(360.0f, 0.0f, 0.0f, 0.0f, 0.0f, 16384.0f);
    }

    wheel_motor_[0].init(&hcan1, 0x200, 0x201, MOTOR_DJI_CONTROL_METHOD_OMEGA, (3591.0f / 187.0f),
                         true);
    wheel_motor_[1].init(&hcan1, 0x200, 0x202, MOTOR_DJI_CONTROL_METHOD_OMEGA, (3591.0f / 187.0f),
                         false);
    wheel_motor_[2].init(&hcan1, 0x200, 0x203, MOTOR_DJI_CONTROL_METHOD_OMEGA, (3591.0f / 187.0f),
                         false);
    wheel_motor_[3].init(&hcan1, 0x200, 0x204, MOTOR_DJI_CONTROL_METHOD_OMEGA, (3591.0f / 187.0f),
                         true);

    gimbal_subscriber_ = MessageCenter::instance().subscribe<GimbalMessage>(kGimbalTopicName);
    cmd_subscriber_ = MessageCenter::instance().subscribe<ChassisCmdMessage>(kCmdChassisTopicName);
}

void Chassis::update_input()
{
    gimbal_subscriber_.update(gimbal_msg_);
    const auto cmd_update = cmd_subscriber_.update(cmd_msg_);

    if (cmd_update)
    {
        limit_acceleration();
        last_cmd_msg_ = cmd_msg_;
    }
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
        last_cmd_msg_ = {};
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
        // 计算旋转速度补偿
        yaw_error -= std::atan(config_.spin_phase_delay * feedback_.omega);
        float sin_yaw = arm_sin_f32(yaw_error);
        float cos_yaw = arm_cos_f32(yaw_error);
        float vx_temp = cmd_msg_.rc_vx;
        float vy_temp = cmd_msg_.rc_vy;
        control_output_.target_velocity_x = vx_temp * cos_yaw + vy_temp * (-sin_yaw);
        control_output_.target_velocity_y = vx_temp * sin_yaw + vy_temp * cos_yaw;
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
        // 轮速缩放
        wheel_omega_scaling();
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

/**
 * @brief 麦轮正解算
 *
 */
void Chassis::mecanum_forward_kinematics()
{
    feedback_.velocity_x = (feedback_.wheel_omega[0] + feedback_.wheel_omega[1] +
                            feedback_.wheel_omega[2] + feedback_.wheel_omega[3]) *
                           config_.wheel_radius / 4.0f;
    feedback_.velocity_y = (feedback_.wheel_omega[0] - feedback_.wheel_omega[1] +
                            feedback_.wheel_omega[2] - feedback_.wheel_omega[3]) *
                           config_.wheel_radius / 4.0f;
    feedback_.omega = (feedback_.wheel_omega[0] - feedback_.wheel_omega[1] -
                       feedback_.wheel_omega[2] + feedback_.wheel_omega[3]) *
                      config_.wheel_radius / config_.rotation_radius / 4.0f;
}

/**
 * @brief 麦轮逆解算
 *
 */
void Chassis::mecanum_inverse_kinematics()
{
    control_output_.wheel_target_omega[0] =
        (control_output_.target_velocity_x + control_output_.target_velocity_y +
         control_output_.target_omega * config_.rotation_radius) /
        config_.wheel_radius;
    control_output_.wheel_target_omega[1] =
        (control_output_.target_velocity_x - control_output_.target_velocity_y -
         control_output_.target_omega * config_.rotation_radius) /
        config_.wheel_radius;
    control_output_.wheel_target_omega[2] =
        (control_output_.target_velocity_x + control_output_.target_velocity_y -
         control_output_.target_omega * config_.rotation_radius) /
        config_.wheel_radius;
    control_output_.wheel_target_omega[3] =
        (control_output_.target_velocity_x - control_output_.target_velocity_y +
         control_output_.target_omega * config_.rotation_radius) /
        config_.wheel_radius;
}

/**
 * @brief 轮速缩放
 *
 */
void Chassis::wheel_omega_scaling()
{
    float scaling_factor = 1.0f;
    for (int i = 0; i < 4; i++)
    {
        if (std::abs(control_output_.wheel_target_omega[i]) > config_.max_wheel_omega)
        {
            scaling_factor =
                std::min(scaling_factor,
                         config_.max_wheel_omega / std::abs(control_output_.wheel_target_omega[i]));
        }
    }
    for (int i = 0; i < 4; i++)
    {
        control_output_.wheel_target_omega[i] *= scaling_factor;
    }
}

/**
 * @brief 有限加速度
 * @note 会修改cmd_msg_的rc_vx和rc_vy
 *
 */
void Chassis::limit_acceleration()
{
    float dx = cmd_msg_.rc_vx - last_cmd_msg_.rc_vx;
    float dy = cmd_msg_.rc_vy - last_cmd_msg_.rc_vy;

    float delta_square = dx * dx + dy * dy;
    float max_acc_delta_square = config_.max_acceleration * config_.max_acceleration * 0.001f * 0.001f;
    float max_dec_delta_square = config_.max_deceleration * config_.max_deceleration * 0.001f * 0.001f;

    bool acc_reverse = last_cmd_msg_.rc_vx * dx + last_cmd_msg_.rc_vy * dy < 0.0f;

    if (!acc_reverse && delta_square > max_acc_delta_square)
    {
        float scale = std::sqrt(max_acc_delta_square / delta_square);
        cmd_msg_.rc_vx = last_cmd_msg_.rc_vx + dx * scale;
        cmd_msg_.rc_vy = last_cmd_msg_.rc_vy + dy * scale;
    }
    else if (acc_reverse && delta_square > max_dec_delta_square)
    {
        float scale = std::sqrt(max_dec_delta_square / delta_square);
        cmd_msg_.rc_vx = last_cmd_msg_.rc_vx + dx * scale;
        cmd_msg_.rc_vy = last_cmd_msg_.rc_vy + dy * scale;
    }
}
/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
