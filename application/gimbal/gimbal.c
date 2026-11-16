/**
 * @file gimbal.c
 * @author anchengc
 * @brief 云台控制
 * @version 0.1
 * @date 2026-06-05 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

/* Includes ------------------------------------------------------------------*/
#include "gimbal.h"
#include "math_tools.h"
#include <math.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
Gimbal_t gimbal;

void Gimbal_Init(void)
{
    gimbal.config.pitch_min_imu_angle = -0.55f;
    gimbal.config.pitch_max_imu_angle = 0.5f;
    gimbal.config.pitch_center_angle = 0.0f;
    gimbal.config.yaw_center_angle = -0.56f;
    gimbal.vision_msg.target_lock = 50; // 49: lock, 50: unlock
    gimbal.cmd_msg.gimbal_mode = GIMBAL_CMD_RELAX;

    // 创建订阅者和发布者

    Create_Subscriber(&gimbal.ins_subscriber, kInsTopicName, sizeof(gimbal.ins_msg));
    Create_Subscriber(&gimbal.vision_subscriber, kVisionTopicName, sizeof(gimbal.vision_msg));
    Create_Subscriber(&gimbal.cmd_subscriber, kCmdGimbalTopicName, sizeof(gimbal.cmd_msg));
    Create_Publisher(&gimbal.gimbal_publisher, kGimbalTopicName, sizeof(Gimbal_Message_t));

    // 云台电机 PID 初始化
    Pid_Init(&gimbal.yaw_angle_pid, 15.0f, 0.0f, 3.0f, 0.0f,
                   0.0f, 0.0f, 0.005f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_FIRST_DISABLE);
    Pid_Init(&gimbal.yaw_omega_pid, 2000.0f, 100000.0f, 0.0f, 0.0f,
                   8000.0f, 16384.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_FIRST_DISABLE);
    Pid_Init(&gimbal.pitch_angle_pid, 30.0f, 0.0f, 2.0f, 0.0f,
                   0.0f, 0.0f, 0.005f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_FIRST_DISABLE);
    Pid_Init(&gimbal.pitch_omega_pid, 2000.0f, 100000.0f, 0.0f, 0.0f,
                   8000.0f, 16384.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_FIRST_DISABLE);

    // 电机初始化
    MotorDji_Init(&gimbal.motor_yaw, &hcan1, 0x1fe, 0x205,
                        MOTOR_DJI_CONTROL_METHOD_CURRENT, 1.0f, 0);
    MotorDji_Init(&gimbal.motor_pitch, &hcan2, 0x1fe, 0x205,
                        MOTOR_DJI_CONTROL_METHOD_CURRENT, 1.0f, 0);

    OneEuroFilter_Init(&gimbal.yaw_filter, 3.0f, 10.0f, 1.0f);
    OneEuroFilter_Init(&gimbal.pitch_filter, 3.0f, 10.0f, 1.0f);
    gimbal.status.mode = GIMBAL_RELAX;

}

/** @brief 更新云台输入 */
void Gimbal_Update_Input(void)
{
    Message_Read(&gimbal.cmd_subscriber, &gimbal.cmd_msg, sizeof(gimbal.cmd_msg));
    Message_Read(&gimbal.vision_subscriber, &gimbal.vision_msg, sizeof(gimbal.vision_msg));
    gimbal.auto_aim_ = !Message_IsTimeout(&gimbal.vision_subscriber, 100) &&
                       gimbal.vision_msg.target_lock == 49 && gimbal.cmd_msg.auto_aim;
}

void Gimbal_Update_Feedback(void)
{
    Message_Read(&gimbal.ins_subscriber, &gimbal.ins_msg, sizeof(gimbal.ins_msg));
    gimbal.feedback.yaw_angle = gimbal.motor_yaw.rx_data.total_angle;
    gimbal.feedback.imu_yaw_angle = gimbal.ins_msg.angle[2];
    gimbal.feedback.imu_pitch_angle = gimbal.ins_msg.angle[1];
    gimbal.feedback.imu_yaw_omega = gimbal.ins_msg.gyro[2];
    gimbal.feedback.imu_pitch_omega = gimbal.ins_msg.gyro[1];
}

void Gimbal_Handle_Safety(void)
{
    if (Message_IsTimeout(&gimbal.cmd_subscriber, 100))
    {
        memset(&gimbal.cmd_msg, 0, sizeof(gimbal.cmd_msg));
        gimbal.auto_aim_ = 0;
    }
}

void Gimbal_Set_Mode(void)
{
    switch (gimbal.cmd_msg.gimbal_mode)
    {
    case GIMBAL_CMD_RELAX:
        gimbal.status.mode = GIMBAL_RELAX;
        gimbal.status.switching = GIMBAL_SWITCH_IDLE;
        break;
    case GIMBAL_CMD_ACTIVE:
        if (gimbal.status.mode != GIMBAL_ACTIVE)
        {
            gimbal.status.switching = GIMBAL_SWITCH_TO_MIDDLE;
            gimbal.control_output.target_yaw_angle = gimbal.feedback.imu_yaw_angle;
        }
        gimbal.status.mode = GIMBAL_ACTIVE;
        break;
    default:
        gimbal.status.mode = GIMBAL_RELAX;
        gimbal.status.switching = GIMBAL_SWITCH_IDLE;
        break;
    }
}

void Gimbal_Control(void)
{
    switch (gimbal.status.switching)
    {
    case GIMBAL_SWITCH_IDLE:
        switch (gimbal.status.mode)
        {
        case GIMBAL_RELAX:
            Pid_SetIntegralError(&gimbal.yaw_angle_pid, 0.0f);
            Pid_SetIntegralError(&gimbal.pitch_angle_pid, 0.0f);
            Pid_SetIntegralError(&gimbal.yaw_omega_pid, 0.0f);
            Pid_SetIntegralError(&gimbal.pitch_omega_pid, 0.0f);
            gimbal.control_output.target_yaw_feedforward_omega = 0.0f;
            gimbal.control_output.target_pitch_feedforward_omega = 0.0f;
            gimbal.control_output.target_yaw_feedforward_acc = 0.0f;
            gimbal.control_output.target_pitch_feedforward_acc = 0.0f;
            break;
        case GIMBAL_ACTIVE:
            if (gimbal.auto_aim_)
            {
                gimbal.control_output.target_yaw_angle =
                    OneEuroFilter_Update(&gimbal.yaw_filter, gimbal.vision_msg.yaw, 0.001f);
                gimbal.control_output.target_pitch_angle =
                    OneEuroFilter_Update(&gimbal.pitch_filter, gimbal.vision_msg.pitch, 0.001f);
                gimbal.control_output.target_yaw_feedforward_omega = gimbal.vision_msg.yaw_vel;
                gimbal.control_output.target_pitch_feedforward_omega = gimbal.vision_msg.pitch_vel;
                gimbal.control_output.target_yaw_feedforward_acc = 0.0f;
                gimbal.control_output.target_pitch_feedforward_acc = 0.0f;
            }
            else
            {
                OneEuroFilter_Reset(&gimbal.yaw_filter);
                OneEuroFilter_Reset(&gimbal.pitch_filter);
                gimbal.control_output.target_yaw_angle += gimbal.cmd_msg.yaw_rate * 0.001f;
                gimbal.control_output.target_pitch_angle += gimbal.cmd_msg.pitch_rate * 0.001f;
                gimbal.control_output.target_yaw_feedforward_omega = 0.0f;
                gimbal.control_output.target_pitch_feedforward_omega = 0.0f;
                gimbal.control_output.target_yaw_feedforward_acc = 0.0f;
                gimbal.control_output.target_pitch_feedforward_acc = 0.0f;
            }
            break;
        default:
            break;
        }
        gimbal.control_output.target_pitch_angle =
            clamp_float(gimbal.control_output.target_pitch_angle,
                        gimbal.config.pitch_min_imu_angle, gimbal.config.pitch_max_imu_angle);
        break;
    case GIMBAL_SWITCH_TO_MIDDLE:
        gimbal.control_output.target_pitch_angle = gimbal.config.pitch_center_angle;
        if (fabsf(gimbal.feedback.imu_pitch_angle - gimbal.config.pitch_center_angle) < 0.1f)
            gimbal.status.switching = GIMBAL_SWITCH_IDLE;
        break;
    default:
        break;
    }
}

void Gimbal_Calculate(void)
{
    static uint8_t mod5 = 5;
    if (gimbal.status.mode == GIMBAL_RELAX)
    {
        gimbal.control_output.target_yaw_current = 0.0f;
        gimbal.control_output.target_pitch_current = 0.0f;
        return;
    }

    gimbal.control_output.target_yaw_error =
        wrap_center_float(gimbal.feedback.imu_yaw_angle -
                          gimbal.control_output.target_yaw_angle, 2.0f * PI);
    if (++mod5 >= 5)
    {
        mod5 = 0;
        Pid_SetTarget(&gimbal.yaw_angle_pid, 0.0f);
        Pid_SetFeedback(&gimbal.yaw_angle_pid, gimbal.control_output.target_yaw_error);
        Pid_Calculate(&gimbal.yaw_angle_pid);
        Pid_SetTarget(&gimbal.pitch_angle_pid, gimbal.control_output.target_pitch_angle);
        Pid_SetFeedback(&gimbal.pitch_angle_pid, gimbal.feedback.imu_pitch_angle);
        Pid_Calculate(&gimbal.pitch_angle_pid);
    }

    gimbal.control_output.target_yaw_omega = Pid_GetOutput(&gimbal.yaw_angle_pid);
    Pid_SetTarget(&gimbal.yaw_omega_pid, gimbal.control_output.target_yaw_omega +
                  gimbal.control_output.target_yaw_feedforward_omega);
    Pid_SetFeedback(&gimbal.yaw_omega_pid, gimbal.feedback.imu_yaw_omega);
    Pid_Calculate(&gimbal.yaw_omega_pid);
    gimbal.control_output.target_pitch_omega = Pid_GetOutput(&gimbal.pitch_angle_pid);
    Pid_SetTarget(&gimbal.pitch_omega_pid, gimbal.control_output.target_pitch_omega +
                  gimbal.control_output.target_pitch_feedforward_omega);
    Pid_SetFeedback(&gimbal.pitch_omega_pid, gimbal.feedback.imu_pitch_omega);
    Pid_Calculate(&gimbal.pitch_omega_pid);

    gimbal.control_output.target_yaw_current = Pid_GetOutput(&gimbal.yaw_omega_pid) +
        gimbal.control_output.target_yaw_feedforward_acc * gimbal.config.yaw_ff_p;
    gimbal.control_output.target_pitch_current = Pid_GetOutput(&gimbal.pitch_omega_pid) +
        gimbal.control_output.target_pitch_feedforward_acc * gimbal.config.pitch_ff_p;
}

void Gimbal_Output(void)
{
    MotorDji_SetTargetCurrent(&gimbal.motor_yaw, gimbal.control_output.target_yaw_current);
    MotorDji_SetTargetCurrent(&gimbal.motor_pitch, gimbal.control_output.target_pitch_current);
    Gimbal_Message_t msg = {.yaw_total_angle = gimbal.feedback.yaw_angle};
    Message_Publish(&gimbal.gimbal_publisher, &msg, sizeof(msg));
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
