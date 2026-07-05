/**
 * @file pid.cpp
 * @author anchengc
 * @brief PID
 * @version 0.1
 * @date 2026-06-05 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "pid.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief PID初始化
 *
 * @param kp P值
 * @param ki I值
 * @param kd D值
 * @param kf 前馈
 * @param integral_output_limit 积分限幅
 * @param output_limit 输出限幅
 * @param dt 时间片长度
 * @param dead_zone 死区误差阈值
 * @param integral_full_error 变速积分误差阈值A
 * @param integral_zero_error 变速积分误差阈值B
 * @param integral_separate_threshold 积分分离误差阈值
 * @param d_first 是否开启微分先行
 */
void Pid::init(float kp, float ki, float kd, float kf, float integral_output_limit,
               float output_limit, float dt, float dead_zone, float integral_full_error,
               float integral_zero_error, float integral_separate_threshold, PidDMode d_first)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    kf_ = kf;
    integral_output_limit_ = integral_output_limit;
    output_limit_ = output_limit;
    dt_ = dt;
    dead_zone_ = dead_zone;
    integral_full_error_ = integral_full_error;
    integral_zero_error_ = integral_zero_error;
    integral_separate_threshold_ = integral_separate_threshold;
    d_first_ = d_first;
}

/**
 * @brief PID调整值, 计算周期与D_T相同
 *
 *
 */
void Pid::calculate()
{
    // P输出
    float p_out = 0.0f;
    // I输出
    float i_out = 0.0f;
    // D输出
    float d_out = 0.0f;
    // F输出
    float f_out = 0.0f;
    // 误差
    float error;
    // 绝对值误差
    float abs_error;
    // 线性变速积分
    float speed_ratio;

    error = target_ - feedback_;
    abs_error = math_abs(error);

    // 判断死区
    if (abs_error < dead_zone_)
    {
        target_ = feedback_;
        error = 0.0f;
        abs_error = 0.0f;
    }
    else if (error > 0.0f && abs_error > dead_zone_)
    {
        error -= dead_zone_;
    }
    else if (error < 0.0f && abs_error > dead_zone_)
    {
        error += dead_zone_;
    }

    // 计算p项

    p_out = kp_ * error;

    // 计算i项

    if (integral_full_error_ == 0.0f && integral_zero_error_ == 0.0f)
    {
        // 非变速积分
        speed_ratio = 1.0f;
    }
    else
    {
        // 变速积分
        if (abs_error <= integral_full_error_)
        {
            speed_ratio = 1.0f;
        }
        else if (integral_full_error_ < abs_error && abs_error < integral_zero_error_)
        {
            speed_ratio =
                (integral_zero_error_ - abs_error) / (integral_zero_error_ - integral_full_error_);
        }
        else if (abs_error >= integral_zero_error_)
        {
            speed_ratio = 0.0f;
        }
    }
    // 积分限幅
    if (integral_output_limit_ != 0.0f)
    {
        math_constrain(&integral_error_, -integral_output_limit_ / ki_,
                       integral_output_limit_ / ki_);
    }
    if (integral_separate_threshold_ == 0.0f)
    {
        // 没有积分分离
        integral_error_ += speed_ratio * dt_ * error;
        i_out = ki_ * integral_error_;
    }
    else
    {
        // 有积分分离
        if (abs_error < integral_separate_threshold_)
        {
            // 不在积分分离区间上
            integral_error_ += speed_ratio * dt_ * error;
            i_out = ki_ * integral_error_;
        }
        else
        {
            // 在积分分离区间上
            integral_error_ = 0.0f;
            i_out = 0.0f;
        }
    }

    // 计算d项

    if (d_first_ == PID_D_FIRST_DISABLE)
    {
        // 没有微分先行
        d_out = kd_ * (error - last_error_) / dt_;
    }
    else
    {
        // 微分先行使能
        d_out = -kd_ * (feedback_ - last_feedback_) / dt_;
    }

    // 计算前馈

    f_out = kf_ * (target_ - last_target_);

    // 计算输出

    output_value_ = p_out + i_out + d_out + f_out;

    // 输出限幅
    if (output_limit_ != 0.0f)
    {
        math_constrain(&output_value_, -output_limit_, output_limit_);
    }

    // 善后工作
    last_feedback_ = feedback_;
    last_target_ = target_;
    last_output_ = output_value_;
    last_error_ = error;
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
