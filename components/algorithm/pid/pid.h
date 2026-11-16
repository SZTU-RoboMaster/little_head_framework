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
#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef enum {
    PID_D_FIRST_DISABLE = 0,
    PID_D_FIRST_ENABLE
} PidDMode_t;

typedef struct {
    /* 参数 */
    float kp;
    float ki;
    float kd;
    float kf;
    float dt;
    float dead_zone;
    float integral_output_limit;
    float output_limit;
    float integral_full_error;
    float integral_zero_error;
    float integral_separate_threshold;
    PidDMode_t d_first;

    /* 本次输入与输出 */
    float target;
    float feedback;
    float output_value;

    /* 上一周期保存的状态 */
    float integral_error;
    float last_feedback;
    float last_target;
    float last_output;
    float last_error;
} Pid_t;

#ifdef __cplusplus
extern "C" {
#endif

bool Pid_Init(Pid_t *pid,
              float kp, float ki, float kd, float kf,
              float integral_output_limit, float output_limit,
              float dt, float dead_zone,
              float integral_full_error, float integral_zero_error,
              float integral_separate_threshold, PidDMode_t d_first);

void Pid_SetTarget(Pid_t *pid, float target);
void Pid_SetFeedback(Pid_t *pid, float feedback);
void Pid_SetIntegralError(Pid_t *pid, float integral_error);
void Pid_Calculate(Pid_t *pid);
float Pid_GetOutput(const Pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif
/************************ COPYRIGHT(C) SZTU-HJ **************************/
