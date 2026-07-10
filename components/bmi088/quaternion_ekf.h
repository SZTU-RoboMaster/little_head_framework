/**
 * @file quaternion_ekf.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-16 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "kalman_filter.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

struct QuaternionEkfState
{
    KalmanFilter_t filter; // 卡尔曼滤波器结构体
    uint8_t converged;
    float q[4];         // 四元数估计值
    float gyro_bias[3]; // 陀螺仪零偏估计值
    float angle[3];     // 欧拉角估计值

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    float dt;                   // 姿态更新周期
    float chi_square;           // 卡方检验检测函数值
    float chi_square_threshold; // 卡方检验阈值
    float lambda;               // 渐消因子
};

/**
 * @brief Specialized
 *
 */
class GravityKf
{
public:
    float gravity_vec_[3]; // 重力向量估计值

    void init(float process_noise, float measure_noise);

    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

protected:
    // 初始化相关常量

    // clang-format off
    // 状态转移雅可比矩阵初始值
    const float f_matrix_[9] = {1, 0, 0,
                                0, 1, 0,
                                0, 0, 1};

    // 后验估计协方差初始值
    const float p_matrix_[9] = {100, 0.1, 0.1,
                                0.1, 100, 0.1,
                                0.1, 0.1, 100};
    // 过程噪声协方差矩阵初始值
    float q_matrix_[9] = {};
    // 测量噪声协方差矩阵初始值
    float r_matrix_[9] = {};
    // 观测雅可比矩阵初始值
    const float h_matrix_[9] = {1, 0, 0,
                                0, 1, 0,
                                0, 0, 1};
    // clang-format on

    // 常量

    // 内部变量

    // 卡尔曼滤波器结构体
    KalmanFilter_t filter_;

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

/**
 * @brief Specialized
 *
 */
class QuaternionEkf
{
public:
    QuaternionEkfState ins_;

    void init(float process_noise1, float process_noise2, float measure_noise, float lambda);

    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

protected:
    // 初始化相关常量

    // clang-format off
    // 状态转移雅可比矩阵初始值
    float f_matrix_[36] = {1, 0, 0, 0, 0, 0,
                           0, 1, 0, 0, 0, 0,
                           0, 0, 1, 0, 0, 0,
                           0, 0, 0, 1, 0, 0,
                           0, 0, 0, 0, 1, 0,
                           0, 0, 0, 0, 0, 1};
    // 后验估计协方差初始值
    float p_matrix_[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                           0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                           0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                           0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                           0.1, 0.1, 0.1, 0.1, 10000, 0.1,
                           0.1, 0.1, 0.1, 0.1, 0.1, 10000};

    // 卡尔曼增益初始值
    float k_matrix_[18] = {};
    // 观测雅可比矩阵初始值
    float h_matrix_[18] = {};
    // clang-format on

    // 常量

    // 内部变量

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数

    static void observe_callback(KalmanFilter_t *kf);
    static void linearize_callback(KalmanFilter_t *kf);
    static void set_h_callback(KalmanFilter_t *kf);
    static void update_xhat_callback(KalmanFilter_t *kf);

    void observe(KalmanFilter_t *kf);

    void linearize_f_and_fade_p(KalmanFilter_t *kf);

    void set_h_matrix(KalmanFilter_t *kf);

    void update_xhat(KalmanFilter_t *kf);
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
