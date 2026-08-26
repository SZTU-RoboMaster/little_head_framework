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
#include "power_controller.h"
#include <algorithm>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/
/**
 * @brief 初始化
 *
 * @param mode 功控模式
 */
void PowerController::init(PowerControlMode mode)
{
    mode_ = mode;

    publisher_ =
        MessageCenter::instance().advertise<PowerControllerMessage>(kPowerControllerTopicName);
    chassis_subscriber_ = MessageCenter::instance().subscribe<ChassisMessage>(kChassisTopicName);
    referee_subscriber_ = MessageCenter::instance().subscribe<RefereeMessage>(kRefereeTopicName);
}

/**
 * @brief 外部调用更新函数
 *
 */
void PowerController::update()
{
    if (update_input())
    {
        allocate_power();
        publisher_.publish(output_msg_);
    }
}

/**
 * @brief 更新输入
 *
 */
bool PowerController::update_input()
{
    referee_subscriber_.update(referee_msg_);
    max_power_ = referee_msg_.chassis_power_limit + 0.2f * (referee_msg_.buffer_energy - 50.0f);
    if (!referee_subscriber_.is_fresh(1000))
    {
        max_power_ = 50.0f;
    }

    // 只在底盘数据更新时才认为需要更新
    return chassis_subscriber_.update(chassis_msg_);
}

/**
 * @brief 分配功率
 *
 */
void PowerController::allocate_power()
{
    if (mode_ == POWER_CONTROL_DISABLE)
    {
        for (int i = 0; i < 4; i++)
        {
            output_msg_.target_current[i] = chassis_msg_.cmd_current[i];
        }
        return;
    }

    float cmd_power[4];
    float omega_error[4];
    float sum_cmd_power = 0.0f;
    float sum_omega_error = 0.0f;
    float allocatable_power = max_power_;
    float sum_required_power = 0.0f;
    // 计算功率
    for (int i = 0; i < 4; i++)
    {
        cmd_power[i] = predict_power(chassis_msg_.cmd_current[i], chassis_msg_.feedback_omega[i]);
        sum_cmd_power += cmd_power[i];
        omega_error[i] = std::abs(chassis_msg_.cmd_omega[i] - chassis_msg_.feedback_omega[i]);
        if (cmd_power[i] < 1e-6f)
        {
            allocatable_power += -cmd_power[i];
        }
        else
        {
            sum_omega_error += omega_error[i];
            sum_required_power += cmd_power[i];
        }
    }
    if (sum_cmd_power > max_power_)
    {
        // 计算置信度
        float error_confidence;
        if (sum_omega_error > omega_err_upper)
        {
            error_confidence = 1.0f;
        }
        else if (sum_omega_error > omega_err_lower)
        {
            error_confidence = std::clamp((sum_omega_error - omega_err_lower) /
                                              (omega_err_upper - omega_err_lower),
                                          0.0f, 1.0f);
        }
        else
        {
            error_confidence = 0.0f;
        }
        // 计算分配功率
        for (int i = 0; i < 4; i++)
        {
            if (cmd_power[i] < 1e-6f)
            {
                output_msg_.target_current[i] = chassis_msg_.cmd_current[i];
                continue;
            }
            float error_weight = sum_omega_error > 1e-6f ? omega_error[i] / sum_omega_error : 0.0f;
            float power_weight = cmd_power[i] / sum_required_power;
            float weight =
                error_confidence * error_weight + (1.0f - error_confidence) * power_weight;
            float allocated_power = weight * allocatable_power;
            // 逆解电流
            output_msg_.target_current[i] =
                current_inverse(chassis_msg_.feedback_omega[i], allocated_power,
                                chassis_msg_.cmd_current[i] > 0.0f);
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            output_msg_.target_current[i] = chassis_msg_.cmd_current[i];
        }
    }
}

/**
 * @brief 预测功率值
 *
 * @param current 电流
 * @param omega 角速度
 * @return float 功率值
 */
float PowerController::predict_power(float current, float omega)
{
    return (k1_ * current * omega + k2_ * std::abs(omega) + k3_ * current * current + a_);
}

/**
 * @brief 电流逆解算
 *
 * @param omega 角速度
 * @param allocated_power 分配功率
 * @param is_positive 电流方向
 * @return float 电流值
 */
float PowerController::current_inverse(float omega, float allocated_power, bool is_positive)
{
    float a = k3_;
    float b = k1_ * omega;
    float c = k2_ * std::abs(omega) + a_ - allocated_power;
    float delta = b * b - 4 * a * c;
    if (delta < 1e-6f)
    {
        return -b / (2 * a);
    }
    else
    {
        if (is_positive)
        {
            return (-b + std::sqrt(delta)) / (2 * a);
        }
        else
        {
            return (-b - std::sqrt(delta)) / (2 * a);
        }
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
