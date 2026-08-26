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

#include "message_center.h"
#include "message_def.h"
/* Exported macros -----------------------------------------------------------*/
// 电机建模系数
static constexpr float k1_ = 3.24956187e-4f;
static constexpr float k2_ = 2.96678362e-1f;
static constexpr float k3_ = 1.77641220e-7f;
static constexpr float a_ = 9.50608963e-1f;

static constexpr float omega_err_lower = 8.0f;
static constexpr float omega_err_upper = 16.0f;
/* Exported types ------------------------------------------------------------*/

enum PowerControlMode
{
    POWER_CONTROL_ENABLE = 0,
    POWER_CONTROL_DISABLE = 1,
};

/**
 * @brief Specialized
 *
 */
class PowerController
{
public:
    void init(PowerControlMode mode);

    void update();

protected:
    // 初始化相关常量
    PowerControlMode mode_ = POWER_CONTROL_ENABLE;

    // 常量

    // 内部变量
    float max_power_ = 50.0f;

    // 读变量
    Subscriber<ChassisMessage> chassis_subscriber_;
    ChassisMessage chassis_msg_;
    Subscriber<RefereeMessage> referee_subscriber_;
    RefereeMessage referee_msg_;

    // 写变量
    Publisher<PowerControllerMessage> publisher_;
    PowerControllerMessage output_msg_;

    // 读写变量

    // 内部函数
    bool update_input();

    void allocate_power();

    float predict_power(float current, float omega);

    float current_inverse(float omega, float allocated_power, bool is_positive);
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
