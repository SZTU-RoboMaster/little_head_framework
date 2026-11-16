/**
 * @file gimbal.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-05 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once
#include <stdbool.h>

/* Includes ------------------------------------------------------------------*/
#include "low_pass_filter.h"
#include "motor_dji.h"
#include "pid.h"

#include "message_center.h"
#include "message_def.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    float yaw_angle;
    float imu_yaw_angle;
    float imu_pitch_angle;
    float imu_yaw_omega;
    float imu_pitch_omega;
}Gimbal_Feedback_t;

typedef struct
{
    // pitch轴最小值
    float pitch_min_imu_angle;
    // pitch轴最大值
    float pitch_max_imu_angle;
    // pitch轴回中值
    float pitch_center_angle;
    // yaw轴回中值
    float yaw_center_angle;
    // yaw轴前馈系数
    float yaw_ff_p;
    // pitch轴前馈系数
    float pitch_ff_p;
}Gimbal_Config_t;

typedef struct
{
    // yaw轴目标角度
    float target_yaw_angle;
    // pitch轴目标角度
    float target_pitch_angle;
    // yaw轴目标角速度
    float target_yaw_omega;
    // pitch轴目标角速度
    float target_pitch_omega;
    // yaw轴前馈角速度
    float target_yaw_feedforward_omega;
    // pitch轴前馈角速度
    float target_pitch_feedforward_omega;
    // yaw轴前馈角加速度
    float target_yaw_feedforward_acc;
    // pitch轴前馈角加速度
    float target_pitch_feedforward_acc;
    // yaw轴目标电流
    float target_yaw_current;
    // pitch轴目标电流
    float target_pitch_current;
    // yaw轴目标角度误差
    float target_yaw_error;
}Gimbal_Output_t;

typedef enum
{
    GIMBAL_RELAX,  // 云台失能
    GIMBAL_ACTIVE, // 云台使能
}GimbalMode_e;

typedef enum
{
    GIMBAL_SWITCH_IDLE,      // 无操作
    GIMBAL_SWITCH_TO_MIDDLE, // 云台回中
}ModeSwitch_e;

typedef struct
{
    GimbalMode_e mode;
    ModeSwitch_e switching;
}Gimbal_Status_e;

/**
 * @brief 云台状态和控制数据
 *
 */
typedef struct
{
    // yaw轴电机
    MotorDji_t motor_yaw;

    // pitch轴电机
    MotorDji_t motor_pitch;

    // 内部变量
    Pid_t yaw_angle_pid;
    Pid_t yaw_omega_pid;
    Pid_t pitch_angle_pid;
    Pid_t pitch_omega_pid;
    OneEuroFilter_t yaw_filter;
    OneEuroFilter_t pitch_filter;

    uint8_t vision_update_;
    uint8_t auto_aim_;

    // 订阅者
    Subscriber_t ins_subscriber;
    Subscriber_t vision_subscriber;
    Subscriber_t cmd_subscriber;

    // 数据
    Ins_Message_t ins_msg;
    Vision_Message_t vision_msg;
    Gimbal_Cmd_Message_t cmd_msg;

    // 发布者
    Publisher_t gimbal_publisher;

    // 云台配置
    Gimbal_Config_t config;
    // 云台反馈
    Gimbal_Feedback_t feedback;
    // 云台状态
    Gimbal_Status_e status;
    // 云台输出
    Gimbal_Output_t control_output;

}Gimbal_t;

/* Exported variables ---------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern Gimbal_t gimbal;

/* Exported function declarations ---------------------------------------------*/

void Gimbal_Init(void);

void Gimbal_Update_Input(void);

void Gimbal_Update_Feedback(void);

void Gimbal_Handle_Safety(void);

void Gimbal_Set_Mode(void);

void Gimbal_Control(void);

void Gimbal_Calculate(void);

void Gimbal_Output(void);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
