/**
 * @file pid.h
 * @author anchengc
 * @brief PID
 * @version 0.1
 * @date 2026-06-05 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 微分先行
 *
 */
enum PidDMode
{
    PID_D_FIRST_DISABLE = 0,
    PID_D_FIRST_ENABLE,
};

/**
 * @brief Reusable, PID算法
 *
 */
class Pid
{
public:
    void init(float kp, float ki, float kd, float kf = 0.0f, float integral_output_limit = 0.0f,
              float output_limit = 0.0f, float dt = 0.001f, float dead_zone = 0.0f,
              float integral_full_error = 0.0f, float integral_zero_error = 0.0f,
              float integral_separate_threshold = 0.0f, PidDMode d_first = PID_D_FIRST_DISABLE);

    inline float get_output();

    inline void set_target(float target);

    inline void set_feedback(float feedback);

    inline void set_integral_error(float integral_error);

    void calculate();

protected:
    // 初始化相关常量

    // PID计时器周期, s
    float dt_;
    // 死区, Error在其绝对值内不输出
    float dead_zone_;
    // 微分先行
    PidDMode d_first_;

    // 常量

    // 内部变量

    // 之前的当前值
    float last_feedback_ = 0.0f;
    // 之前的目标值
    float last_target_ = 0.0f;
    // 之前的输出值
    float last_output_ = 0.0f;
    // 前向误差
    float last_error_ = 0.0f;

    // 读变量

    // 输出值
    float output_value_ = 0.0f;

    // 写变量

    // PID的P
    float kp_ = 0.0f;
    // PID的I
    float ki_ = 0.0f;
    // PID的D
    float kd_ = 0.0f;
    // 前馈
    float kf_ = 0.0f;

    // 积分限幅, 0为不限制
    float integral_output_limit_ = 0;
    // 输出限幅, 0为不限制
    float output_limit_ = 0;

    // 变速积分定速内段阈值, 0为不限制
    float integral_full_error_ = 0.0f;
    // 变速积分变速区间, 0为不限制
    float integral_zero_error_ = 0.0f;
    // 积分分离阈值，需为正数, 0为不限制
    float integral_separate_threshold_ = 0.0f;

    // 目标值
    float target_ = 0.0f;
    // 当前值
    float feedback_ = 0.0f;

    // 读写变量

    // 积分值
    float integral_error_ = 0.0f;

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
inline float Pid::get_output()
{
    return (output_value_);
}

/**
 * @brief 设定目标值
 *
 * @param target 目标值
 */
inline void Pid::set_target(float target)
{
    target_ = target;
}

/**
 * @brief 设定当前值
 *
 * @param feedback 当前值
 */
inline void Pid::set_feedback(float feedback)
{
    feedback_ = feedback;
}

/**
 * @brief 设定积分, 一般用于积分清零
 *
 * @param __Set_Integral_Error 积分值
 */
inline void Pid::set_integral_error(float integral_error)
{
    integral_error_ = integral_error;
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
