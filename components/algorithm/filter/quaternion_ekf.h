/**
 * @file quaternion_ekf.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-16 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "kalman_filter.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    KalmanFilter_t filter; // 卡尔曼滤波器结构体

    float q[4];         // 四元数估计值
    float gyro_bias[3]; // 陀螺仪零偏估计值
    float angle[3];     // 欧拉角估计值

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    uint8_t converged;  // 发散flag
    uint8_t stable;     // 稳定flag
    uint32_t error_cnt; // 卡方校验出错次数

    float dt;                   // 姿态更新周期
    float chi_square;           // 卡方检验检测函数值
    float chi_square_threshold; // 卡方检验阈值
    float lambda;               // 渐消因子

    float adaptive_gain_scale;      // 自适应增益缩放系数
    float orientation_arccosine[3]; // 方向余弦角
} QuaternionEkfState_t;

typedef struct
{
    float gravity_vec[3]; // 重力向量估计值

    float f_matrix[9]; // 状态转移雅可比矩阵
    float p_matrix[9]; // 后验估计协方差
    float q_matrix[9]; // 过程噪声协方差
    float r_matrix[9]; // 测量噪声协方差
    float h_matrix[9]; // 观测雅可比矩阵
    KalmanFilter_t filter;
} GravityKf_t;

typedef struct
{
    QuaternionEkfState_t ins;
    float f_matrix[36]; // 状态转移雅可比矩阵
    float p_matrix[36]; // 后验估计协方差
    float k_matrix[18]; // 卡尔曼增益
    float h_matrix[18]; // 观测雅可比矩阵
} QuaternionEkf_t;

/* Exported function declarations ---------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void GravityKf_Init(GravityKf_t *self, float process_noise, float measure_noise);
void GravityKf_Update(GravityKf_t *self, float gx, float gy, float gz,
                      float ax, float ay, float az, float dt);
void QuaternionEkf_Init(QuaternionEkf_t *self, float process_noise1,
                        float process_noise2, float measure_noise, float lambda);
void QuaternionEkf_Update(QuaternionEkf_t *self, float gx, float gy, float gz,
                          float ax, float ay, float az, float dt);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
