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
#include <string.h>
#include "math_tools.h"

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
bool Pid_Init(Pid_t *pid,
              float kp, float ki, float kd, float kf,
              float integral_output_limit, float output_limit,
              float dt, float dead_zone,
              float integral_full_error, float integral_zero_error,
              float integral_separate_threshold, PidDMode_t d_first)
{
    if (pid == NULL || dt <= 0.0f ||
        (integral_output_limit != 0.0f && ki == 0.0f)) {
        return false;
        }

    memset(pid, 0, sizeof(*pid));  /* 清除上次运行留下的积分、误差等状态 */

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kf = kf;
    pid->integral_output_limit = integral_output_limit;
    pid->output_limit = output_limit;
    pid->dt = dt;
    pid->dead_zone = dead_zone;
    pid->integral_full_error = integral_full_error;
    pid->integral_zero_error = integral_zero_error;
    pid->integral_separate_threshold = integral_separate_threshold;
    pid->d_first = d_first;
    return true;
}

void Pid_SetTarget(Pid_t *pid, float target)
{
    if (pid != NULL) pid->target = target;
}

void Pid_SetFeedback(Pid_t *pid, float feedback)
{
    if (pid != NULL) pid->feedback = feedback;
}

void Pid_SetIntegralError(Pid_t *pid, float integral_error)
{
    if (pid != NULL) pid->integral_error = integral_error;
}

float Pid_GetOutput(const Pid_t *pid)
{
    return pid != NULL ? pid->output_value : 0.0f;
}


/**
 * @brief PID计算, 计算周期与D_T相同
 *
 *
 */
void Pid_Calculate(Pid_t *pid)
{
    if (pid == NULL || pid->dt <= 0.0f)
    {
        return;
    }
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

    error = pid->target - pid->feedback;
    abs_error = fabsf(error);

    // 判断死区
    if (abs_error < pid->dead_zone)
    {
        pid->target = pid->feedback;
        error = 0.0f;
        abs_error = 0.0f;
    }
    else if (error > 0.0f && abs_error > pid->dead_zone)
    {
        error -= pid->dead_zone;
    }
    else if (error < 0.0f && abs_error > pid->dead_zone)
    {
        error += pid->dead_zone;
    }

    // 计算p项
    p_out = pid->kp * error;

    // 计算i项
    if (pid->integral_full_error == 0.0f && pid->integral_zero_error == 0.0f)
    {
        // 非变速积分
        speed_ratio = 1.0f;
    }
    else
    {
        // 变速积分
        if (abs_error <= pid->integral_full_error)
        {
            speed_ratio = 1.0f;
        }
        else if (pid->integral_full_error < abs_error && abs_error < pid->integral_zero_error)
        {
            speed_ratio =
                (pid->integral_zero_error - abs_error) /
                (pid->integral_zero_error - pid->integral_full_error);
        }
        else if (abs_error >= pid->integral_zero_error)
        {
            speed_ratio = 0.0f;
        }
        else
        {
            speed_ratio = 0.0f;
        }
    }
    // 积分限幅
    if (pid->integral_output_limit != 0.0f)
    {
        pid->integral_error = clamp_float(pid->integral_error,
                                          -pid->integral_output_limit / pid->ki,
                                          pid->integral_output_limit / pid->ki);

    }
    if (pid->integral_separate_threshold == 0.0f)
    {
        // 没有积分分离
        pid->integral_error += speed_ratio * pid->dt * error;
        i_out = pid->ki * pid->integral_error;
    }
    else
    {
        // 有积分分离
        if (abs_error < pid->integral_separate_threshold)
        {
            // 不在积分分离区间上
            pid->integral_error += speed_ratio * pid->dt * error;
            i_out = pid->ki * pid->integral_error;
        }
        else
        {
            // 在积分分离区间上
            pid->integral_error = 0.0f;
            i_out = 0.0f;
        }
    }

    // 计算d项
    if (pid->d_first == PID_D_FIRST_DISABLE)
    {
        // 没有微分先行
        d_out = pid->kd * (error - pid->last_error) / pid->dt;
    }
    else
    {
        // 微分先行使能
        d_out = -pid->kd * (pid->feedback - pid->last_feedback) / pid->dt;
    }

    // 计算前馈
    f_out = pid->kf * (pid->target - pid->last_target);

    // 计算输出
    pid->output_value = p_out + i_out + d_out + f_out;

    // 输出限幅
    if (pid->output_limit != 0.0f)
    {
        pid->output_value = clamp_float(pid->output_value,
                                         -pid->output_limit, pid->output_limit);
    }

    // 更新
    pid->last_feedback = pid->feedback;
    pid->last_target = pid->target;
    pid->last_output = pid->output_value;
    pid->last_error = error;
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
