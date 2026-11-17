/**
 * @file chassis.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-05 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

/* Includes ------------------------------------------------------------------*/
#include "chassis.h"
#include "math_tools.h"
#include <math.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Chassis_t chassis;

/* Private function declarations ---------------------------------------------*/
static void Chassis_Mecanum_Inverse_Kinematics(void);
static void Chassis_Mecanum_Forward_Kinematics(void);
static void Chassis_Wheel_Omega_Scaling(void);
static void Chassis_Limit_Acceleration(void);

/* function prototypes -------------------------------------------------------*/
void Chassis_Init(void)
{
    chassis.config.wheel_radius = 0.07689f;
    chassis.config.half_length = 0.193560273f;
    chassis.config.half_width = 0.167982291f;
    chassis.config.rotation_radius = chassis.config.half_length + chassis.config.half_width;
    chassis.config.gimbal_yaw_offset = -0.56f;
    chassis.config.spin_vw = 7.0f;
    chassis.config.spin_phase_delay = 0.03f;
    chassis.config.max_wheel_omega = 46.0f;
    chassis.config.max_acceleration = 3.0f;
    chassis.config.max_deceleration = 6.0f;

    // 底盘角速度PID
    Pid_Init(&chassis.omega_pid, 7.0f, 0.0f, 0.0f, 0.0f,
             0.0f, 0.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_FIRST_DISABLE);

    // 轮向电机初始化
    for (int i = 0; i < 4; i++)
    {
        Pid_Init(&chassis.wheel_omega_pid[i], 360.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 16384.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_FIRST_DISABLE);
    }

    MotorDji_Init(&chassis.wheel_motor[0], &hfdcan1, 0x200, 0x201,
                  MOTOR_DJI_CONTROL_METHOD_CURRENT, 3591.0f / 187.0f, 1);
    MotorDji_Init(&chassis.wheel_motor[1], &hfdcan1, 0x200, 0x202,
                  MOTOR_DJI_CONTROL_METHOD_CURRENT, 3591.0f / 187.0f, 0);
    MotorDji_Init(&chassis.wheel_motor[2], &hfdcan1, 0x200, 0x203,
                  MOTOR_DJI_CONTROL_METHOD_CURRENT, 3591.0f / 187.0f, 0);
    MotorDji_Init(&chassis.wheel_motor[3], &hfdcan1, 0x200, 0x204,
                  MOTOR_DJI_CONTROL_METHOD_CURRENT, 3591.0f / 187.0f, 1);

    // Topic 已由 robot_sdk 注册
    Create_Publisher(&chassis.chassis_publisher, kChassisTopicName, sizeof(Chassis_Message_t));
    Create_Subscriber(&chassis.gimbal_subscriber, kGimbalTopicName, sizeof(chassis.gimbal_msg));
    Create_Subscriber(&chassis.cmd_subscriber, kCmdChassisTopicName, sizeof(chassis.cmd_msg));
    Create_Subscriber(&chassis.power_controller_subscriber, kPowerControllerTopicName,
                      sizeof(chassis.power_controller_msg));
    chassis.mode = CHASSIS_RELAX;
}

void Chassis_Update_Input(void)
{
    Message_Read(&chassis.gimbal_subscriber, &chassis.gimbal_msg, sizeof(chassis.gimbal_msg));
    Message_Read(&chassis.power_controller_subscriber, &chassis.power_controller_msg,
                 sizeof(chassis.power_controller_msg));

    uint32_t previous_update = chassis.cmd_subscriber.last_read_update_count;
    Chassis_Cmd_Message_t incoming_cmd;
    if (Message_Read(&chassis.cmd_subscriber, &incoming_cmd, sizeof(incoming_cmd)) &&
        chassis.cmd_subscriber.last_read_update_count != previous_update)
    {
        chassis.cmd_msg = incoming_cmd;
        Chassis_Limit_Acceleration();
        chassis.last_cmd_msg = chassis.cmd_msg;
    }
}

void Chassis_Update_Feedback(void)
{
    for (int i = 0; i < 4; i++)
    {
        chassis.feedback.wheel_omega[i] = chassis.wheel_motor[i].rx_data.omega;
    }
    Chassis_Mecanum_Forward_Kinematics();
    chassis.feedback.gimbal_yaw = chassis.gimbal_msg.yaw_total_angle;
}

void Chassis_Handle_Safety(void)
{
    // 安全处理
    if (Message_IsTimeout(&chassis.cmd_subscriber, 100))
    {
        memset(&chassis.cmd_msg, 0, sizeof(chassis.cmd_msg));
    }
}

void Chassis_Set_Mode(void)
{
    // 设置模式
    switch (chassis.cmd_msg.chassis_mode)
    {
    case CHASSIS_CMD_RELAX:  chassis.mode = CHASSIS_RELAX;  break;
    case CHASSIS_CMD_ONLY:   chassis.mode = CHASSIS_ONLY;   break;
    case CHASSIS_CMD_FOLLOW: chassis.mode = CHASSIS_FOLLOW; break;
    case CHASSIS_CMD_SPIN:   chassis.mode = CHASSIS_SPIN;   break;
    default:                chassis.mode = CHASSIS_RELAX;  break;
    }
}

void Chassis_Control(void)
{
    // 控制
    float yaw_error = 0.0f;
    switch (chassis.mode)
    {
    case CHASSIS_RELAX:
        memset(&chassis.last_cmd_msg, 0, sizeof(chassis.last_cmd_msg));
        chassis.control_output.target_velocity_x = 0.0f;
        chassis.control_output.target_velocity_y = 0.0f;
        chassis.control_output.target_omega = 0.0f;
        break;
    case CHASSIS_ONLY:
        chassis.control_output.target_velocity_x = chassis.cmd_msg.rc_vx;
        chassis.control_output.target_velocity_y = chassis.cmd_msg.rc_vy;
        chassis.control_output.target_omega = chassis.cmd_msg.rc_vw;
        break;
    case CHASSIS_FOLLOW:
        yaw_error = wrap_center_float(chassis.feedback.gimbal_yaw -
                                      chassis.config.gimbal_yaw_offset, 2.0f * PI);
        Pid_SetTarget(&chassis.omega_pid, 0.0f);
        Pid_SetFeedback(&chassis.omega_pid, yaw_error);
        Pid_Calculate(&chassis.omega_pid);
        chassis.control_output.target_velocity_x = chassis.cmd_msg.rc_vx;
        chassis.control_output.target_velocity_y = chassis.cmd_msg.rc_vy;
        chassis.control_output.target_omega = -Pid_GetOutput(&chassis.omega_pid);
        break;
    case CHASSIS_SPIN:
    {
        yaw_error = wrap_center_float(chassis.feedback.gimbal_yaw -
                                      chassis.config.gimbal_yaw_offset, 2.0f * PI);
        // 计算旋转速度补偿
        yaw_error -= atanf(chassis.config.spin_phase_delay * chassis.feedback.omega);
        float sin_yaw = arm_sin_f32(yaw_error);
        float cos_yaw = arm_cos_f32(yaw_error);
        float vx_temp = chassis.cmd_msg.rc_vx;
        float vy_temp = chassis.cmd_msg.rc_vy;
        chassis.control_output.target_velocity_x = vx_temp * cos_yaw - vy_temp * sin_yaw;
        chassis.control_output.target_velocity_y = vx_temp * sin_yaw + vy_temp * cos_yaw;
        chassis.control_output.target_omega = chassis.config.spin_vw;
        break;
    }
    default:
        break;
    }
}

void Chassis_Solve(void)
{
    if (chassis.mode == CHASSIS_RELAX)
    {
        for (int i = 0; i < 4; i++)
            chassis.control_output.wheel_target_current[i] = 0.0f;
        return;
    }

    // 解算
    Chassis_Mecanum_Inverse_Kinematics();
    // 轮速缩放
    Chassis_Wheel_Omega_Scaling();
    for (int i = 0; i < 4; i++)
    {
        Pid_SetTarget(&chassis.wheel_omega_pid[i], chassis.control_output.wheel_target_omega[i]);
        Pid_SetFeedback(&chassis.wheel_omega_pid[i], chassis.feedback.wheel_omega[i]);
        Pid_Calculate(&chassis.wheel_omega_pid[i]);
        chassis.control_output.wheel_target_current[i] =
            Pid_GetOutput(&chassis.wheel_omega_pid[i]);
    }
}

void Chassis_Output(void)
{
    Chassis_Message_t chassis_msg = {0};
    for (int i = 0; i < 4; i++)
    {
        if (chassis.mode == CHASSIS_RELAX)
            MotorDji_SetTargetCurrent(&chassis.wheel_motor[i], 0.0f);
        else
            MotorDji_SetTargetCurrent(&chassis.wheel_motor[i],
                                      chassis.power_controller_msg.target_current[i]);

        chassis_msg.feedback_omega[i] = chassis.feedback.wheel_omega[i];
        chassis_msg.cmd_omega[i] = chassis.control_output.wheel_target_omega[i];
        chassis_msg.cmd_current[i] = chassis.control_output.wheel_target_current[i];
    }
    Message_Publish(&chassis.chassis_publisher, &chassis_msg, sizeof(chassis_msg));
}

/** @brief 麦轮正解算 */
static void Chassis_Mecanum_Forward_Kinematics(void)
{
    chassis.feedback.velocity_x = (chassis.feedback.wheel_omega[0] +
                                   chassis.feedback.wheel_omega[1] +
                                   chassis.feedback.wheel_omega[2] +
                                   chassis.feedback.wheel_omega[3]) *
                                  chassis.config.wheel_radius / 4.0f;
    chassis.feedback.velocity_y = (chassis.feedback.wheel_omega[0] -
                                   chassis.feedback.wheel_omega[1] +
                                   chassis.feedback.wheel_omega[2] -
                                   chassis.feedback.wheel_omega[3]) *
                                  chassis.config.wheel_radius / 4.0f;
    chassis.feedback.omega = (chassis.feedback.wheel_omega[0] -
                             chassis.feedback.wheel_omega[1] -
                             chassis.feedback.wheel_omega[2] +
                             chassis.feedback.wheel_omega[3]) *
                            chassis.config.wheel_radius / chassis.config.rotation_radius / 4.0f;
}

/** @brief 麦轮逆解算 */
static void Chassis_Mecanum_Inverse_Kinematics(void)
{
    chassis.control_output.wheel_target_omega[0] =
        (chassis.control_output.target_velocity_x + chassis.control_output.target_velocity_y +
         chassis.control_output.target_omega * chassis.config.rotation_radius) /
        chassis.config.wheel_radius;
    chassis.control_output.wheel_target_omega[1] =
        (chassis.control_output.target_velocity_x - chassis.control_output.target_velocity_y -
         chassis.control_output.target_omega * chassis.config.rotation_radius) /
        chassis.config.wheel_radius;
    chassis.control_output.wheel_target_omega[2] =
        (chassis.control_output.target_velocity_x + chassis.control_output.target_velocity_y -
         chassis.control_output.target_omega * chassis.config.rotation_radius) /
        chassis.config.wheel_radius;
    chassis.control_output.wheel_target_omega[3] =
        (chassis.control_output.target_velocity_x - chassis.control_output.target_velocity_y +
         chassis.control_output.target_omega * chassis.config.rotation_radius) /
        chassis.config.wheel_radius;
}

/** @brief 轮速缩放 */
static void Chassis_Wheel_Omega_Scaling(void)
{
    float scaling_factor = 1.0f;
    for (int i = 0; i < 4; i++)
    {
        float omega = fabsf(chassis.control_output.wheel_target_omega[i]);
        if (omega > chassis.config.max_wheel_omega)
            scaling_factor = fminf(scaling_factor, chassis.config.max_wheel_omega / omega);
    }
    for (int i = 0; i < 4; i++)
        chassis.control_output.wheel_target_omega[i] *= scaling_factor;
}

/**
 * @brief 有限加速度
 * @note 会修改cmd_msg的rc_vx和rc_vy
 */
static void Chassis_Limit_Acceleration(void)
{
    float dx = chassis.cmd_msg.rc_vx - chassis.last_cmd_msg.rc_vx;
    float dy = chassis.cmd_msg.rc_vy - chassis.last_cmd_msg.rc_vy;
    float delta_square = dx * dx + dy * dy;
    float max_acc_delta_square = chassis.config.max_acceleration *
                                 chassis.config.max_acceleration * 0.001f * 0.001f;
    float max_dec_delta_square = chassis.config.max_deceleration *
                                 chassis.config.max_deceleration * 0.001f * 0.001f;
    bool is_positive = chassis.last_cmd_msg.rc_vx * dx +
                       chassis.last_cmd_msg.rc_vy * dy > 0.0f;
    if (is_positive && delta_square > max_acc_delta_square)
    {
        float scale = sqrtf(max_acc_delta_square / delta_square);
        chassis.cmd_msg.rc_vx = chassis.last_cmd_msg.rc_vx + dx * scale;
        chassis.cmd_msg.rc_vy = chassis.last_cmd_msg.rc_vy + dy * scale;
    }
    else if (!is_positive && delta_square > max_dec_delta_square)
    {
        float scale = sqrtf(max_dec_delta_square / delta_square);
        chassis.cmd_msg.rc_vx = chassis.last_cmd_msg.rc_vx + dx * scale;
        chassis.cmd_msg.rc_vy = chassis.last_cmd_msg.rc_vy + dy * scale;
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
