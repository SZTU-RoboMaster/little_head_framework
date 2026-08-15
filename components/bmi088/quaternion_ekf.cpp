/**
 * @file quaternion_ekf.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-16 0.1 初版
 * @note ref: https://zhuanlan.zhihu.com/p/454155643
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "quaternion_ekf.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 重力向量估计KF初始化
 *
 * @param process_noise 过程噪声
 * @param measure_noise 观测噪声
 */
void GravityKf::init(float process_noise, float measure_noise)
{
    for (uint8_t i = 0; i < 9; i += 4)
    {
        // 初始化过程噪声与量测噪声
        q_matrix_[i] = process_noise;
        r_matrix_[i] = measure_noise;
    }

    Kalman_Filter_Init(&filter_, 3, 0, 3); // 状态向量3维 无控制部分 测量向量3维
    // filter_.User_Func0_f = gEstimateKF_Tuning;
    memcpy(filter_.F_data, f_matrix_, sizeof(f_matrix_));
    memcpy(filter_.P_data, p_matrix_, sizeof(p_matrix_));
    memcpy(filter_.Q_data, q_matrix_, sizeof(q_matrix_));
    memcpy(filter_.R_data, r_matrix_, sizeof(r_matrix_));
    memcpy(filter_.H_data, h_matrix_, sizeof(h_matrix_));
}

/**
 * @brief 重力向量估计KF更新
 *
 * @param process_noise 过程噪声
 * @param measure_noise 观测噪声
 */
void GravityKf::update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // 空间换时间 避免重复运算
    static float gxdt, gydt, gzdt;
    gxdt = gx * dt;
    gydt = gy * dt;
    gzdt = gz * dt;

    // 由于本例中状态转移矩阵为时变矩阵
    // 需要在卡尔曼滤波器更新前更新转移矩阵F的值
    filter_.F_data[1] = gzdt;
    filter_.F_data[2] = -gydt;

    filter_.F_data[3] = -gzdt;
    filter_.F_data[5] = gxdt;

    filter_.F_data[6] = gydt;
    filter_.F_data[7] = -gxdt;

    // 卡尔曼滤波器测量值更新
    // 不一定写在滤波器更新函数之前，也可写在与传感器通信的回调函数中
    filter_.MeasuredVector[0] = ax;
    filter_.MeasuredVector[1] = ay;
    filter_.MeasuredVector[2] = az;

    // 卡尔曼滤波器更新函数
    Kalman_Filter_Update(&filter_);

    // 提取估计值
    for (uint8_t i = 0; i < 3; i++)
    {
        gravity_vec_[i] = filter_.FilteredValue[i];
    }
}

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 */
void QuaternionEkf::init(float process_noise1, float process_noise2, float measure_noise,
                         float lambda)
{

    ins_.Q1 = process_noise1;
    ins_.Q2 = process_noise2;
    ins_.R = measure_noise;
    ins_.chi_square_threshold = 0.01f;
    ins_.converged = 0;
    if (lambda > 1)
    {
        lambda = 1;
    }
    ins_.lambda = lambda;
    Kalman_Filter_Init(&ins_.filter, 6, 0, 3);
    // 姿态初始化
    ins_.filter.xhat_data[0] = 1;
    ins_.filter.xhat_data[1] = 0;
    ins_.filter.xhat_data[2] = 0;
    ins_.filter.xhat_data[3] = 0;
    // 自定义函数初始化,用于扩展或增加kf的基础功能
    ins_.filter.User_Ptr = this;
    ins_.filter.User_Func0_f = observe_callback;
    ins_.filter.User_Func1_f = linearize_callback;
    ins_.filter.User_Func2_f = set_h_callback;
    ins_.filter.User_Func3_f = update_xhat_callback;
    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    ins_.filter.SkipEq3 = true;
    ins_.filter.SkipEq4 = true;

    memcpy(ins_.filter.F_data, f_matrix_, sizeof(f_matrix_));
    memcpy(ins_.filter.P_data, p_matrix_, sizeof(p_matrix_));
}

/**
 * @brief Quaternion EKF update
 * @param[in] gyro x y z in rad/s
 * @param[in] accel x y z
 * @param[in] update period in s
 */
void QuaternionEkf::update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // 0.5(Ohm-Ohm^bias)*delta_t,用于更新工作点处的状态转移F矩阵
    static float halfgxdt, halfgydt, halfgzdt;
    static float accel_inv_norm;
    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    18    19    20    21    22    23
    24    25    26    27    28    29
    30    31    32    33    34    35
    */
    ins_.dt = dt;

    halfgxdt = 0.5f * (gx - ins_.gyro_bias[0]) * dt;
    halfgydt = 0.5f * (gy - ins_.gyro_bias[1]) * dt;
    halfgzdt = 0.5f * (gz - ins_.gyro_bias[2]) * dt;

    // 初始化F矩阵为单位阵
    memcpy(ins_.filter.F_data, f_matrix_, sizeof(f_matrix_));
    // 此部分设定状态转移矩阵F的左上角部分
    // 4x4子矩阵,即0.5(Ohm-Ohm^bias)*delta_t,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    ins_.filter.F_data[1] = -halfgxdt;
    ins_.filter.F_data[2] = -halfgydt;
    ins_.filter.F_data[3] = -halfgzdt;

    ins_.filter.F_data[6] = halfgxdt;
    ins_.filter.F_data[8] = halfgzdt;
    ins_.filter.F_data[9] = -halfgydt;

    ins_.filter.F_data[12] = halfgydt;
    ins_.filter.F_data[13] = -halfgzdt;
    ins_.filter.F_data[15] = halfgxdt;

    ins_.filter.F_data[18] = halfgzdt;
    ins_.filter.F_data[19] = halfgydt;
    ins_.filter.F_data[20] = -halfgxdt;

    // 归一化加速度向量作为量测向量
    accel_inv_norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ins_.filter.MeasuredVector[0] = ax * accel_inv_norm;
    ins_.filter.MeasuredVector[1] = ay * accel_inv_norm;
    ins_.filter.MeasuredVector[2] = az * accel_inv_norm;

    // 设置Q,R矩阵
    ins_.filter.Q_data[0] = ins_.Q1 * ins_.dt;
    ins_.filter.Q_data[7] = ins_.Q1 * ins_.dt;
    ins_.filter.Q_data[14] = ins_.Q1 * ins_.dt;
    ins_.filter.Q_data[21] = ins_.Q1 * ins_.dt;
    ins_.filter.Q_data[28] = ins_.Q2 * ins_.dt;
    ins_.filter.Q_data[35] = ins_.Q2 * ins_.dt;
    ins_.filter.R_data[0] = ins_.R;
    ins_.filter.R_data[4] = ins_.R;
    ins_.filter.R_data[8] = ins_.R;

    // 卡尔曼滤波器更新
    Kalman_Filter_Update(&ins_.filter);

    // 估计结果导出
    ins_.q[0] = ins_.filter.FilteredValue[0];
    ins_.q[1] = ins_.filter.FilteredValue[1];
    ins_.q[2] = ins_.filter.FilteredValue[2];
    ins_.q[3] = ins_.filter.FilteredValue[3];
    ins_.gyro_bias[0] = ins_.filter.FilteredValue[4];
    ins_.gyro_bias[1] = ins_.filter.FilteredValue[5];
    ins_.gyro_bias[2] = -0.0019f;

    // 四元数反解欧拉角
    ins_.angle[0] = atan2f(2.0f * (ins_.q[0] * ins_.q[1] + ins_.q[2] * ins_.q[3]),
                           2.0f * (ins_.q[0] * ins_.q[0] + ins_.q[3] * ins_.q[3]) - 1.0f);
    ins_.angle[1] = asinf(-2.0f * (ins_.q[1] * ins_.q[3] - ins_.q[0] * ins_.q[2]));
    ins_.angle[2] = atan2f(2.0f * (ins_.q[0] * ins_.q[3] + ins_.q[1] * ins_.q[2]),
                           2.0f * (ins_.q[0] * ins_.q[0] + ins_.q[1] * ins_.q[1]) - 1.0f);
}

void QuaternionEkf::observe(KalmanFilter_t *kf)
{
    memcpy(p_matrix_, kf->P_data, sizeof(p_matrix_));
    memcpy(k_matrix_, kf->K_data, sizeof(k_matrix_));
    memcpy(h_matrix_, kf->H_data, sizeof(h_matrix_));
}

void QuaternionEkf::linearize_f_and_fade_p(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;
    static float q_inv_norm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // 四元数归一化
    q_inv_norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf->xhatminus_data[i] *= q_inv_norm;
    }
    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    18    19    20    21    22    23
    24    25    26    27    28    29
    30    31    32    33    34    35
    */
    // 补充F矩阵
    kf->F_data[4] = q1 * ins_.dt / 2;
    kf->F_data[5] = q2 * ins_.dt / 2;

    kf->F_data[10] = -q0 * ins_.dt / 2;
    kf->F_data[11] = q3 * ins_.dt / 2;

    kf->F_data[16] = -q3 * ins_.dt / 2;
    kf->F_data[17] = -q0 * ins_.dt / 2;

    kf->F_data[22] = q2 * ins_.dt / 2;
    kf->F_data[23] = -q1 * ins_.dt / 2;
}

void QuaternionEkf::set_h_matrix(KalmanFilter_t *kf)
{
    static float doubleq0, doubleq1, doubleq2, doubleq3;
    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    */

    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];
    // 复位H矩阵为0矩阵
    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    // 设置H矩阵
    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

void QuaternionEkf::update_xhat(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;

    // 计算残差方差 inv(H·P'(k)·HT + R)
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus =
        Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1); // temp_matrix1 = inv(H·P'(k)·HT + R)

    // 计算h(xhat'(k))
    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];
    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 计算残差z(k) - h(xhat'(k))
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector,
                                    &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // 卡方检验 计算检验函数r
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus =
        Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1,
                        &kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus =
        Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->temp_matrix.numRows = 1;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_vector1, &kf->temp_matrix);

    // 检测函数
    ins_.chi_square = kf->temp_matrix.pData[0];
    if (ins_.chi_square < 0.1f * ins_.chi_square_threshold)
    {
        ins_.converged = 1;
    }
    if (ins_.chi_square > ins_.chi_square_threshold && ins_.converged)
    {
        // 未通过卡方检验 仅预测
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
        kf->SkipEq5 = true;
        return;
    }
    else
    {
        // 应用渐消因子
        kf->P_data[28] /= ins_.lambda;
        kf->P_data[35] /= ins_.lambda;
        kf->SkipEq5 = false;
    }

    // 通过卡方检验，进行量测更新
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus =
        Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1,
                                    &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))
    kf->temp_vector.pData[3] = 0;                      // 应用M矩阵
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector,
                               &kf->xhat); // xhat = xhat'(k) + M·K(k)·(z(k) - h(xhat'(k)))
}

void QuaternionEkf::observe_callback(KalmanFilter_t *kf)
{
    auto *self = static_cast<QuaternionEkf *>(kf->User_Ptr);
    self->observe(kf);
}

void QuaternionEkf::linearize_callback(KalmanFilter_t *kf)
{
    auto *self = static_cast<QuaternionEkf *>(kf->User_Ptr);
    self->linearize_f_and_fade_p(kf);
}

void QuaternionEkf::set_h_callback(KalmanFilter_t *kf)
{
    auto *self = static_cast<QuaternionEkf *>(kf->User_Ptr);
    self->set_h_matrix(kf);
}

void QuaternionEkf::update_xhat_callback(KalmanFilter_t *kf)
{
    auto *self = static_cast<QuaternionEkf *>(kf->User_Ptr);
    self->update_xhat(kf);
}
/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
