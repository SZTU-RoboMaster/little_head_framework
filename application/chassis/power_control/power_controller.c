/**
 * @file power_controller.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-08-26 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

/* Includes ------------------------------------------------------------------*/
#include "power_controller.h"
#include "math_tools.h"
#include <math.h>

/* Private macros ------------------------------------------------------------*/
// 电机建模系数
static const float kPowerK1 = 3.24956187e-4f;
static const float kPowerK2 = 2.96678362e-1f;
static const float kPowerK3 = 1.77641220e-7f;
static const float kPowerA = 9.50608963e-1f;
static const float kOmegaErrLower = 8.0f;
static const float kOmegaErrUpper = 16.0f;

/* Private variables ---------------------------------------------------------*/
PowerController_t power_controller;

/* Private function declarations ---------------------------------------------*/
static bool PowerController_Update_Input(void);
static void PowerController_Allocate_Power(void);
static float PowerController_Predict_Power(float current, float omega);
static float PowerController_Current_Inverse(float omega, float allocated_power, bool is_positive);

/**
 * @brief 初始化
 * @param mode 功控模式
 */
void PowerController_Init(PowerControlMode_e mode)
{
    power_controller.mode = mode;
    power_controller.max_power = 50.0f;

    // Topic 已由 robot_sdk 注册
    Create_Publisher(&power_controller.power_controller_publisher,
                     kPowerControllerTopicName, sizeof(Power_Controller_Message_t));
    Create_Subscriber(&power_controller.chassis_subscriber,
                      kChassisTopicName, sizeof(power_controller.chassis_msg));
    Create_Subscriber(&power_controller.referee_subscriber,
                      kRefereeTopicName, sizeof(power_controller.referee_msg));
}

/**
 * @brief 外部调用更新函数
 */
void PowerController_Update(void)
{
    if (PowerController_Update_Input())
    {
        PowerController_Allocate_Power();
        Message_Publish(&power_controller.power_controller_publisher,
                        &power_controller.output_msg, sizeof(power_controller.output_msg));
    }
}

/**
 * @brief 更新输入
 */
static bool PowerController_Update_Input(void)
{
    Message_Read(&power_controller.referee_subscriber, &power_controller.referee_msg,
                 sizeof(power_controller.referee_msg));
    power_controller.max_power = power_controller.referee_msg.chassis_power_limit +
        0.2f * (power_controller.referee_msg.buffer_energy - 50.0f);
    if (Message_IsTimeout(&power_controller.referee_subscriber, 1000))
        power_controller.max_power = 50.0f;

    // 只在底盘数据更新时才认为需要更新
    uint32_t previous_update = power_controller.chassis_subscriber.last_read_update_count;
    return Message_Read(&power_controller.chassis_subscriber, &power_controller.chassis_msg,
                        sizeof(power_controller.chassis_msg)) &&
           power_controller.chassis_subscriber.last_read_update_count != previous_update;
}

/**
 * @brief 分配功率
 */
static void PowerController_Allocate_Power(void)
{
    if (power_controller.mode == POWER_CONTROL_DISABLE)
    {
        for (int i = 0; i < 4; i++)
            power_controller.output_msg.target_current[i] =
                power_controller.chassis_msg.cmd_current[i];
        return;
    }

    float cmd_power[4];
    float omega_error[4];
    float sum_cmd_power = 0.0f;
    float sum_omega_error = 0.0f;
    float allocatable_power = power_controller.max_power;
    float sum_required_power = 0.0f;
    // 计算功率
    for (int i = 0; i < 4; i++)
    {
        cmd_power[i] = PowerController_Predict_Power(
            power_controller.chassis_msg.cmd_current[i],
            power_controller.chassis_msg.feedback_omega[i]);
        sum_cmd_power += cmd_power[i];
        omega_error[i] = fabsf(power_controller.chassis_msg.cmd_omega[i] -
                               power_controller.chassis_msg.feedback_omega[i]);
        if (cmd_power[i] < 1e-6f)
            allocatable_power -= cmd_power[i];
        else
        {
            sum_omega_error += omega_error[i];
            sum_required_power += cmd_power[i];
        }
    }

    if (sum_cmd_power > power_controller.max_power)
    {
        // 计算置信度
        float error_confidence;
        if (sum_omega_error > kOmegaErrUpper)
            error_confidence = 1.0f;
        else if (sum_omega_error > kOmegaErrLower)
            error_confidence = clamp_float(
                (sum_omega_error - kOmegaErrLower) / (kOmegaErrUpper - kOmegaErrLower),
                0.0f, 1.0f);
        else
            error_confidence = 0.0f;

        // 计算分配功率
        for (int i = 0; i < 4; i++)
        {
            if (cmd_power[i] < 1e-6f)
            {
                power_controller.output_msg.target_current[i] =
                    power_controller.chassis_msg.cmd_current[i];
                continue;
            }
            float error_weight = sum_omega_error > 1e-6f ?
                omega_error[i] / sum_omega_error : 0.0f;
            float power_weight = cmd_power[i] / sum_required_power;
            float weight = error_confidence * error_weight +
                           (1.0f - error_confidence) * power_weight;
            float allocated_power = weight * allocatable_power;
            // 逆解电流
            power_controller.output_msg.target_current[i] =
                PowerController_Current_Inverse(
                    power_controller.chassis_msg.feedback_omega[i], allocated_power,
                    power_controller.chassis_msg.cmd_current[i] > 0.0f);
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
            power_controller.output_msg.target_current[i] =
                power_controller.chassis_msg.cmd_current[i];
    }
}

/**
 * @brief 预测功率值
 * @param current 电流
 * @param omega 角速度
 * @return float 功率值
 */
static float PowerController_Predict_Power(float current, float omega)
{
    return kPowerK1 * current * omega + kPowerK2 * fabsf(omega) +
           kPowerK3 * current * current + kPowerA;
}

/**
 * @brief 电流逆解算
 * @param omega 角速度
 * @param allocated_power 分配功率
 * @param is_positive 电流方向
 * @return float 电流值
 */
static float PowerController_Current_Inverse(float omega, float allocated_power, bool is_positive)
{
    float a = kPowerK3;
    float b = kPowerK1 * omega;
    float c = kPowerK2 * fabsf(omega) + kPowerA - allocated_power;
    float delta = b * b - 4.0f * a * c;
    if (delta < 1e-6f)
        return -b / (2.0f * a);
    if (is_positive)
        return (-b + sqrtf(delta)) / (2.0f * a);
    return (-b - sqrtf(delta)) / (2.0f * a);
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
