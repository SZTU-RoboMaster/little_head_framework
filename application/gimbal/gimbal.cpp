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
#include "gimbal.h"

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
void Gimbal::init()
{
    // 云台电机PID初始化
    yaw_angle_pid_.init(6.0f, 0.0f, 0.6f, 0.0f, 0.0f, 0.0f, 0.005f);
    yaw_omega_pid_.init(2000.0f, 40000.0f, 0.0f, 0.0f, 8000.0f, 16384.0f);
    pitch_angle_pid_.init(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.005f);
    pitch_omega_pid_.init(2000.0f, 40000.0f, 0.0f, 0.0f, 8000.0f, 16384.0f);
    // 电机初始化
    motor_yaw_.init(&hcan1, 0x1fe, 0x205, MOTOR_DJI_CONTROL_METHOD_CURRENT, 1.0f);
    motor_pitch_.init(&hcan2, 0x1fe, 0x205, MOTOR_DJI_CONTROL_METHOD_CURRENT, 1.0f);

    // 初始化云台状态
    status_.mode = GIMBAL_RELAX;

    publisher_ = MessageCenter::instance().advertise<GimbalMessage>(kGimbalTopicName);
    ins_subscriber_ = MessageCenter::instance().subscribe<InsMessage>(kInsTopicName);
    vision_subscriber_ = MessageCenter::instance().subscribe<VisionMessage>(kVisionTopicName);
    cmd_subscriber_ = MessageCenter::instance().subscribe<GimbalCmdMessage>(kCmdGimbalTopicName);
}

/**
 * @brief 更新云台输入
 *
 */
void Gimbal::update_input()
{
    vision_subscriber_.update(vision_msg_);
    vision_online_flag_ = vision_subscriber_.is_fresh(100);
    cmd_subscriber_.update(cmd_msg_);
}

void Gimbal::update_feedback()
{
    ins_subscriber_.update(ins_msg_);

    feedback_.yaw_angle = motor_yaw_.rx_data_.total_angle;

    feedback_.imu_yaw_angle = ins_msg_.angle[2];
    feedback_.imu_pitch_angle = ins_msg_.angle[1];
    feedback_.imu_yaw_omega = ins_msg_.gyro[2];
    feedback_.imu_pitch_omega = ins_msg_.gyro[1];
}

void Gimbal::handle_safety()
{
    // 安全处理
    if (!cmd_subscriber_.is_fresh(100))
    {
        cmd_msg_ = {};
    }
}

void Gimbal::set_mode()
{
    switch (cmd_msg_.gimbal_mode)
    {
    case GIMBAL_CMD_RELAX:
        status_.mode = GIMBAL_RELAX;
        status_.switching = GIMBAL_SWITCH_IDLE;
        break;
    case GIMBAL_CMD_ACTIVE:
        if (status_.mode != GIMBAL_ACTIVE)
        {
            status_.switching = GIMBAL_SWITCH_TO_MIDDLE;
            control_output_.target_yaw_angle = feedback_.imu_yaw_angle;
        }
        status_.mode = GIMBAL_ACTIVE;
        break;
    default:
        break;
    }
}

void Gimbal::control()
{
    switch (status_.switching)
    {
    case GIMBAL_SWITCH_IDLE:

        // 正常控制
        switch (status_.mode)
        {
        case GIMBAL_RELAX:
            // 云台失能，电机不输出
            yaw_angle_pid_.set_integral_error(0.0f);
            pitch_angle_pid_.set_integral_error(0.0f);
            yaw_omega_pid_.set_integral_error(0.0f);
            pitch_omega_pid_.set_integral_error(0.0f);
            control_output_.target_yaw_feedforward_omega = 0.0f;
            control_output_.target_pitch_feedforward_omega = 0.0f;
            control_output_.target_yaw_feedforward_acc = 0.0f;
            control_output_.target_pitch_feedforward_acc = 0.0f;
            break;

        case GIMBAL_ACTIVE:
            // 云台使能，电机输出
            if (vision_online_flag_ && vision_msg_.target_lock == 49)
            {
                control_output_.target_yaw_angle = vision_msg_.yaw;
                control_output_.target_pitch_angle = vision_msg_.pitch;
                control_output_.target_yaw_feedforward_omega = vision_msg_.yaw_vel;
                control_output_.target_pitch_feedforward_omega = vision_msg_.pitch_vel;
                control_output_.target_yaw_feedforward_acc = 0.0f;
                control_output_.target_pitch_feedforward_acc = 0.0f;
            }
            else
            {
                control_output_.target_yaw_angle += cmd_msg_.yaw_rate * 0.001f;
                control_output_.target_pitch_angle += cmd_msg_.pitch_rate * 0.001f;
                control_output_.target_yaw_feedforward_omega = 0.0f;
                control_output_.target_pitch_feedforward_omega = 0.0f;
                control_output_.target_yaw_feedforward_acc = 0.0f;
                control_output_.target_pitch_feedforward_acc = 0.0f;
            }
            break;

        default:
            break;
        }

        // 输出限幅
        control_output_.target_pitch_angle =
            std::clamp(control_output_.target_pitch_angle, -0.55f, 0.5f);
        break;

    case GIMBAL_SWITCH_TO_MIDDLE:
        control_output_.target_pitch_angle = config_.pitch_center_angle;

        if (std::abs(feedback_.imu_pitch_angle - config_.pitch_center_angle) < 0.1f)
        {
            status_.switching = GIMBAL_SWITCH_IDLE;
        }
        break;

    default:
        break;
    }
}

void Gimbal::calculate()
{
    static uint8_t mod5 = 5;

    if (status_.mode == GIMBAL_RELAX)
    {
        control_output_.target_yaw_current = 0.0f;
        control_output_.target_pitch_current = 0.0f;
    }
    else
    {
        control_output_.target_yaw_error =
            wrap_center((feedback_.imu_yaw_angle - control_output_.target_yaw_angle), (2.0f * PI));
        if (++mod5 >= 5)
        {
            mod5 = 0;
            yaw_angle_pid_.set_target(0.0f);
            yaw_angle_pid_.set_feedback(control_output_.target_yaw_error);
            yaw_angle_pid_.calculate();

            pitch_angle_pid_.set_target(control_output_.target_pitch_angle);
            pitch_angle_pid_.set_feedback(feedback_.imu_pitch_angle);
            pitch_angle_pid_.calculate();
        }

        control_output_.target_yaw_omega = yaw_angle_pid_.get_output();
        yaw_omega_pid_.set_target(control_output_.target_yaw_omega +
                                  control_output_.target_yaw_feedforward_omega);
        yaw_omega_pid_.set_feedback(feedback_.imu_yaw_omega);
        yaw_omega_pid_.calculate();
        control_output_.target_pitch_omega = pitch_angle_pid_.get_output();
        pitch_omega_pid_.set_target(control_output_.target_pitch_omega +
                                    control_output_.target_pitch_feedforward_omega);
        pitch_omega_pid_.set_feedback(feedback_.imu_pitch_omega);
        pitch_omega_pid_.calculate();

        control_output_.target_yaw_current =
            yaw_omega_pid_.get_output() +
            control_output_.target_yaw_feedforward_acc * config_.yaw_ff_p;
        control_output_.target_pitch_current =
            pitch_omega_pid_.get_output() +
            control_output_.target_pitch_feedforward_acc * config_.pitch_ff_p;
    }
}

void Gimbal::output()
{
    motor_yaw_.set_target_current(control_output_.target_yaw_current);
    motor_pitch_.set_target_current(control_output_.target_pitch_current);

    GimbalMessage msg = {.yaw_total_angle = feedback_.yaw_angle};
    publisher_.publish(msg);
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
