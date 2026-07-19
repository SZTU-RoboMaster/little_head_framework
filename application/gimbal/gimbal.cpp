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
#include "INS_task.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Gimbal gimbal;

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
}

/**
 * @brief 更新云台输入
 *
 */
void Gimbal::update_input()
{
    input_.ch_3 = dr16_->data_.ch_3;
    input_.ch_2 = -dr16_->data_.ch_2;
    input_.sw_2 = dr16_->data_.sw_2;
}

void Gimbal::update_feedback()
{
    feedback_.yaw_angle = motor_yaw_.rx_data_.total_angle;
    feedback_.imu_yaw_angle = ins.quaternion_ekf_.ins_.angle[2];
    feedback_.imu_pitch_angle = ins.quaternion_ekf_.ins_.angle[1];

    feedback_.imu_yaw_omega = ins.bmi088_.rx_data_.gyro[2];
    feedback_.imu_pitch_omega = ins.bmi088_.rx_data_.gyro[1];
}

void Gimbal::handle_safety()
{
    // 安全处理
}

void Gimbal::set_mode()
{
    switch (input_.sw_2)
    {
    case 2:
        status_.mode = GIMBAL_RELAX;
        status_.switching = GIMBAL_SWITCH_IDLE;
        break;
    case 3:
        status_.mode = GIMBAL_RELAX;
        status_.switching = GIMBAL_SWITCH_IDLE;
        break;
    case 1:
        if (status_.mode != GIMBAL_ACTIVE)
        {
            status_.switching = GIMBAL_SWITCH_TO_MIDDLE;
        }
        break;
    default:
        break;
    }
}

void Gimbal::update_control_state()
{
    if (status_.switching != GIMBAL_SWITCH_IDLE)
    {
        return;
    }

    // 键鼠，遥控器，自瞄输入到底听谁的
    // 目前先只听遥控器的，后续可以加优先级判断

    // 屏蔽控制量
    // 最终输出控制量为input_.final
    switch (status_.mode)
    {
    case GIMBAL_RELAX:
        // 云台失能，电机不输出
        yaw_angle_pid_.set_integral_error(0.0f);
        pitch_angle_pid_.set_integral_error(0.0f);
        break;
    case GIMBAL_ACTIVE:
        // 云台使能，电机输出
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

        // 后续把输入量改成control_judge的输出量
        control_output_.target_pitch_angle += cubic_map(input_.ch_3 / 660.0f, 0.5f) * 5.0f * 0.001f;
        control_output_.target_yaw_angle += cubic_map(input_.ch_2 / 660.0f, 0.5f) * 4.0f * 0.001f;
        control_output_.target_yaw_error =
            wrap_center((feedback_.imu_yaw_angle - control_output_.target_yaw_angle), (2.0f * PI));
        break;

    case GIMBAL_SWITCH_TO_MIDDLE:
        control_output_.target_pitch_angle = config_.pitch_center_angle;
        control_output_.target_yaw_angle = config_.yaw_center_angle;

        control_output_.target_yaw_error =
            wrap_center((feedback_.yaw_angle - control_output_.target_yaw_angle), (2.0f * PI));
        if (std::abs(feedback_.imu_pitch_angle - config_.pitch_center_angle) < 0.1f &&
            std::abs(wrap_center((feedback_.yaw_angle - config_.yaw_center_angle), (2.0f * PI))) <
                0.1f)
        {
            status_.mode = GIMBAL_ACTIVE;
            status_.switching = GIMBAL_SWITCH_IDLE;
            control_output_.target_yaw_angle = feedback_.imu_yaw_angle;
        }
        break;

    default:
        break;
    }
}

void Gimbal::calculate()
{
    static uint8_t mod5 = 5;

    // 输出限幅
    control_output_.target_pitch_angle =
        std::clamp(control_output_.target_pitch_angle, -0.55f, 0.5f);

    if (status_.mode == GIMBAL_RELAX && status_.switching == GIMBAL_SWITCH_IDLE)
    {
        control_output_.target_yaw_current = 0.0f;
        control_output_.target_pitch_current = 0.0f;
    }
    else
    {
        if (mod5++ >= 5)
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
        yaw_omega_pid_.set_target(control_output_.target_yaw_omega);
        yaw_omega_pid_.set_feedback(feedback_.imu_yaw_omega);
        yaw_omega_pid_.calculate();
        control_output_.target_pitch_omega = pitch_angle_pid_.get_output();
        pitch_omega_pid_.set_target(control_output_.target_pitch_omega);
        pitch_omega_pid_.set_feedback(feedback_.imu_pitch_omega);
        pitch_omega_pid_.calculate();

        control_output_.target_yaw_current = yaw_omega_pid_.get_output();
        control_output_.target_pitch_current = pitch_omega_pid_.get_output();
    }
}

void Gimbal::output()
{
    motor_yaw_.set_target_current(control_output_.target_yaw_current);
    motor_pitch_.set_target_current(control_output_.target_pitch_current);
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
