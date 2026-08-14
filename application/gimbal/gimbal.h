/**
 * @file gimbal.h
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

#include "message_center.h"
#include "message_def.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

struct GimbalFeedback
{
    float yaw_angle;
    float imu_yaw_angle;
    float imu_pitch_angle;
    float imu_yaw_omega;
    float imu_pitch_omega;
};

struct GimbalConfig
{
    // pitch轴最小值
    float pitch_min_angle;
    // pitch轴最大值
    float pitch_max_angle;
    // pitch轴KP值
    float pitch_kp = 30.0f;
    // pitch轴KD值
    float pitch_kd = 3.0f;
    // pitch轴回中值
    float pitch_center_angle = 0.0f;
    // yaw轴回中值
    float yaw_center_angle = -0.56f;
    // yaw轴KP值
    float yaw_kp;
    // yaw轴KD值
    float yaw_kd;
    // yaw轴前馈系数
    float yaw_ff_p = 0.0f;
    // pitch轴前馈系数
    float pitch_ff_p = 0.0f;
};

struct GimbalOutput
{
    // yaw轴目标角度
    float target_yaw_angle = 0.0f;
    // pitch轴目标角度
    float target_pitch_angle = 0.0f;
    // yaw轴目标角速度
    float target_yaw_omega = 0.0f;
    // pitch轴目标角速度
    float target_pitch_omega = 0.0f;
    // yaw轴前馈角速度
    float target_yaw_feedforward_omega = 0.0f;
    // pitch轴前馈角速度
    float target_pitch_feedforward_omega = 0.0f;
    // yaw轴前馈角加速度
    float target_yaw_feedforward_acc = 0.0f;
    // pitch轴前馈角加速度
    float target_pitch_feedforward_acc = 0.0f;
    // yaw轴目标电流
    float target_yaw_current = 0.0f;
    // pitch轴目标电流
    float target_pitch_current = 0.0f;
    // yaw轴目标角度误差
    float target_yaw_error = 0.0f;
};

enum GimbalMode
{
    GIMBAL_RELAX,  // 云台失能
    GIMBAL_ACTIVE, // 云台使能
};

enum ModeSwitch
{
    GIMBAL_SWITCH_IDLE,      // 无操作
    GIMBAL_SWITCH_TO_MIDDLE, // 云台回中
};

struct GimbalStatus
{
    GimbalMode mode;
    ModeSwitch switching;
};

/**
 * @brief Specialized, 云台类
 *
 */
class Gimbal
{
public:
    // yaw轴电机
    MotorDji motor_yaw_;
    Pid yaw_angle_pid_;
    Pid yaw_omega_pid_;

    // pitch轴电机
    MotorDji motor_pitch_;
    Pid pitch_angle_pid_;
    Pid pitch_omega_pid_;

    void init();

    void update_input();

    void update_feedback();

    void handle_safety();

    void set_mode();

    void update_control_state();

    void control();

    void calculate();

    void output();

protected:
    // 初始化相关常量

    // 常量

    // 云台配置
    GimbalConfig config_;

    // 内部变量
    uint8_t vision_online_flag_ = 0;

    // 读变量
    Subscriber<InsMessage> ins_subscriber_;
    InsMessage ins_msg_;
    Subscriber<VisionMessage> vision_subscriber_;
    VisionMessage vision_msg_;
    Subscriber<GimbalCmdMessage> cmd_subscriber_;
    GimbalCmdMessage cmd_msg_;

    // 写变量
    Publisher<GimbalMessage> publisher_;

    // 云台反馈
    GimbalFeedback feedback_;
    // 云台状态
    GimbalStatus status_;

    // 读写变量

    // 云台输出
    GimbalOutput control_judge_;
    GimbalOutput control_output_;

    // 内部函数
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
