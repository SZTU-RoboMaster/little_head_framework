/**
 * @file chassis.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-05 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "motor_dji.h"
#include "pid.h"
#include "message_center.h"
#include "message_def.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    // 轮子半径
    float wheel_radius;
    // 前后轮到底盘中心的距离
    float half_length;
    // 左右轮到底盘中心的距离
    float half_width;
    // 旋转半径
    float rotation_radius;
    // 云台偏航角偏移
    float gimbal_yaw_offset;
    // 小陀螺vw
    float spin_vw;
    // 小陀螺相位滞后的等效时间常数
    // TODO: 有没有必要考虑功控的影响
    float spin_phase_delay;
    // 轮向电机最大角速度
    float max_wheel_omega;
    // 平移最大加速度
    float max_acceleration;
    // 平移最大减速度
    float max_deceleration;
} Chassis_Config_t;

typedef struct
{
    float wheel_omega[4];
    float velocity_x;
    float velocity_y;
    float omega;
    float gimbal_yaw; // yaw电机角度
} Chassis_Feedback_t;

typedef struct
{
    // 目标速度X
    float target_velocity_x;
    // 目标速度Y
    float target_velocity_y;
    // 目标角速度
    float target_omega;
    // 轮向电机目标角速度
    float wheel_target_omega[4];
    // 轮向电机目标电流
    float wheel_target_current[4];
} Chassis_Output_t;

typedef enum
{
    CHASSIS_RELAX,  // 底盘失能
    CHASSIS_ONLY,   // 底盘独立
    CHASSIS_FOLLOW, // 底盘跟随
    CHASSIS_SPIN,   // 小陀螺
} ChassisMode_e;

/**
 * @brief 底盘控制数据
 */
typedef struct
{
    // 底盘角速度PID
    Pid_t omega_pid;
    // 轮向电机
    MotorDji_t wheel_motor[4];
    // 轮向电机角速度PID
    Pid_t wheel_omega_pid[4];

    // 底盘配置
    Chassis_Config_t config;
    // 内部变量
    Chassis_Cmd_Message_t last_cmd_msg;

    // 订阅者
    Subscriber_t gimbal_subscriber;
    Subscriber_t cmd_subscriber;
    Subscriber_t power_controller_subscriber;

    // 数据
    Gimbal_Message_t gimbal_msg;
    Chassis_Cmd_Message_t cmd_msg;
    Power_Controller_Message_t power_controller_msg;

    // 发布者
    Publisher_t chassis_publisher;

    // 底盘反馈
    Chassis_Feedback_t feedback;
    // 底盘状态
    ChassisMode_e mode;
    // 底盘输出
    Chassis_Output_t control_output;
} Chassis_t;

/* Exported variables ---------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern Chassis_t chassis;

/* Exported function declarations ---------------------------------------------*/
void Chassis_Init(void);
void Chassis_Update_Input(void);
void Chassis_Update_Feedback(void);
void Chassis_Handle_Safety(void);
void Chassis_Set_Mode(void);
void Chassis_Control(void);
void Chassis_Solve(void);
void Chassis_Output(void);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
