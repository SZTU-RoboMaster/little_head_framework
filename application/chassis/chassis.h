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
#include "math_tools.h"
#include "motor_dji.h"
#include "pid.h"

#include "message_center.h"
#include "message_def.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

struct ChassisConfig
{
    // 轮子半径
    float wheel_radius = 0.07689f;

    // 前后轮到底盘中心的距离
    float half_length = 0.193560273f;

    // 左右轮到底盘中心的距离
    float half_width = 0.167982291f;

    // 旋转半径
    float rotation_radius = half_length + half_width; // 0.361542564f

    // 云台偏航角偏移
    float gimbal_yaw_offset = -0.56f;

    // 小陀螺vw
    float spin_vw = 7.0f;

    // 小陀螺相位滞后的等效时间常数
    float spin_phase_delay = 0.06f;

    // 轮向电机最大角速度
    float max_wheel_omega = 46.0f;

    // 平移最大加速度
    float max_acceleration = 3.0f;

    // 平移最大减速度
    float max_deceleration = 5.0f;
};

struct ChassisFeedback
{
    float wheel_omega[4];
    float velocity_x;
    float velocity_y;
    float omega;
    float gimbal_yaw; // yaw电机角度
};

struct ChassisOutput
{
    // 目标速度X
    float target_velocity_x = 0.0f;
    // 目标速度Y
    float target_velocity_y = 0.0f;
    // 目标角速度
    float target_omega = 0.0f;

    // 轮向电机目标角速度
    float wheel_target_omega[4];

    // 轮向电机目标电流
    float wheel_target_current[4];
};

enum ChassisMode
{
    CHASSIS_RELAX,  // 底盘失能
    CHASSIS_ONLY,   // 底盘独立
    CHASSIS_FOLLOW, // 底盘跟随
    CHASSIS_SPIN,   // 小陀螺
};

/**
 * @brief Specialized, 底盘类
 *
 */
class Chassis
{
public:
    // 底盘角速度PID
    Pid omega_pid_;

    // 轮向电机
    MotorDji wheel_motor_[4];

    void init();

    void update_input();

    void update_feedback();

    void handle_safety();

    void set_mode();

    void control();

    void solve();

    void output();

protected:
    // 初始化相关常量

    // 常量

    // 底盘配置
    static constexpr ChassisConfig config_{};

    // 内部变量
    ChassisCmdMessage last_cmd_msg_;

    // 读变量
    Subscriber<GimbalMessage> gimbal_subscriber_;
    GimbalMessage gimbal_msg_;
    Subscriber<ChassisCmdMessage> cmd_subscriber_;
    ChassisCmdMessage cmd_msg_;

    // 写变量

    // 底盘反馈
    ChassisFeedback feedback_;
    // 底盘状态
    ChassisMode mode_;

    // 读写变量

    // 底盘输出
    ChassisOutput control_output_;

    // 内部函数

    // 麦轮逆解算
    void mecanum_inverse_kinematics();
    // 麦轮正解算
    void mecanum_forward_kinematics();
    // 轮速缩放
    void wheel_omega_scaling();
    // 有限加速度
    void limit_acceleration();
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
