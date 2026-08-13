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

struct ChassisInput
{
    int16_t ch_1; // x
    int16_t ch_0; // y
    int16_t ch_2; // yaw
    uint8_t sw_1; // 底盘控制方式
    uint8_t sw_2; // 底盘控制方式
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

struct ChassisConfig
{
    // 轮子半径
    float wheel_radius = 0.075f;

    // 轮子到底盘中心的距离
    float wheel_to_center_distance = 0.59463f;

    // 云台偏航角偏移
    float gimbal_yaw_offset = -0.56f;

    // 小陀螺vw
    float spin_vw = 7.0f;
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
    ChassisConfig config_;

    // 内部变量

    // 读变量
    Subscriber<GimbalMessage> gimbal_subscriber_;
    GimbalMessage gimbal_message_;
    Subscriber<Dr16Message> dr16_subscriber_;
    Dr16Message dr16_message_;

    // 写变量

    // 底盘输入
    ChassisInput input_;
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
    // 麦轮解算
    void mecanum_forward_kinematics();
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
