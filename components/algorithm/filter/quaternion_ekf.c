/**
 * @file quaternion_ekf.c
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
#include "math_tools.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
static void QuaternionEkf_Observe(QuaternionEkf_t *self, KalmanFilter_t *kf);
static void QuaternionEkf_LinearizeFAndFadeP(QuaternionEkf_t *self, KalmanFilter_t *kf);
static void QuaternionEkf_SetHMatrix(QuaternionEkf_t *self, KalmanFilter_t *kf);
static void QuaternionEkf_UpdateXhat(QuaternionEkf_t *self, KalmanFilter_t *kf);
static void QuaternionEkf_ObserveCallback(KalmanFilter_t *kf);
static void QuaternionEkf_LinearizeCallback(KalmanFilter_t *kf);
static void QuaternionEkf_SetHCallback(KalmanFilter_t *kf);
static void QuaternionEkf_UpdateXhatCallback(KalmanFilter_t *kf);


/* function prototypes -------------------------------------------------------*/

/**
 * @brief 重力向量估计KF初始化
 *
 * @param process_noise 过程噪声
 * @param measure_noise 观测噪声
 */
void GravityKf_Init(GravityKf_t *self, float process_noise, float measure_noise)
{
    static const float initial_f[9] = {1,0,0, 0,1,0, 0,0,1};
    static const float initial_p[9] = {100,0.1f,0.1f, 0.1f,100,0.1f, 0.1f,0.1f,100};
    memcpy(self->f_matrix, initial_f, sizeof(initial_f));
    memcpy(self->p_matrix, initial_p, sizeof(initial_p));
    memcpy(self->h_matrix, initial_f, sizeof(initial_f));
    for (int i = 0; i < 9; i += 4)
    {
        // 初始化过程噪声与量测噪声
        self->q_matrix[i] = process_noise;
        self->r_matrix[i] = measure_noise;
    }

    Kalman_Filter_Init(&self->filter, 3, 0, 3); // 状态向量3维 无控制部分 测量向量3维
    // self->filter.User_Func0_f = gEstimateKF_Tuning;
    memcpy(self->filter.F_data, self->f_matrix, sizeof(self->f_matrix));
    memcpy(self->filter.P_data, self->p_matrix, sizeof(self->p_matrix));
    memcpy(self->filter.Q_data, self->q_matrix, sizeof(self->q_matrix));
    memcpy(self->filter.R_data, self->r_matrix, sizeof(self->r_matrix));
    memcpy(self->filter.H_data, self->h_matrix, sizeof(self->h_matrix));
}

/**
 * @brief 重力向量估计KF更新
 *
 * @param process_noise 过程噪声
 * @param measure_noise 观测噪声
 */
void GravityKf_Update(GravityKf_t *self, float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // 空间换时间 避免重复运算
    static float gxdt, gydt, gzdt;
    gxdt = gx * dt;
    gydt = gy * dt;
    gzdt = gz * dt;

    // 由于本例中状态转移矩阵为时变矩阵
    // 需要在卡尔曼滤波器更新前更新转移矩阵F的值
    self->filter.F_data[1] = gzdt;
    self->filter.F_data[2] = -gydt;

    self->filter.F_data[3] = -gzdt;
    self->filter.F_data[5] = gxdt;

    self->filter.F_data[6] = gydt;
    self->filter.F_data[7] = -gxdt;

    // 卡尔曼滤波器测量值更新
    // 不一定写在滤波器更新函数之前，也可写在与传感器通信的回调函数中
    self->filter.MeasuredVector[0] = ax;
    self->filter.MeasuredVector[1] = ay;
    self->filter.MeasuredVector[2] = az;

    // 卡尔曼滤波器更新函数
    Kalman_Filter_Update(&self->filter);

    // 提取估计值
    for (int i = 0; i < 3; i++)
    {
        self->gravity_vec[i] = self->filter.FilteredValue[i];
    }
}

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 */
void QuaternionEkf_Init(QuaternionEkf_t *self, float process_noise1, float process_noise2, float measure_noise,
                         float lambda)
{
    static const float initial_f[36] = {1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1};
    static const float initial_p[36] = {100000,0.1f,0.1f,0.1f,0.1f,0.1f, 0.1f,100000,0.1f,0.1f,0.1f,0.1f, 0.1f,0.1f,100000,0.1f,0.1f,0.1f, 0.1f,0.1f,0.1f,100000,0.1f,0.1f, 0.1f,0.1f,0.1f,0.1f,100,0.1f, 0.1f,0.1f,0.1f,0.1f,0.1f,100};
    memcpy(self->f_matrix, initial_f, sizeof(initial_f));
    memcpy(self->p_matrix, initial_p, sizeof(initial_p));
    self->ins.Q1 = process_noise1;
    self->ins.Q2 = process_noise2;
    self->ins.R = measure_noise;
    self->ins.chi_square_threshold = 1e-8f;
    self->ins.converged = 0;
    if (lambda > 1)
    {
        lambda = 1;
    }
    self->ins.lambda = lambda;
    Kalman_Filter_Init(&self->ins.filter, 6, 0, 3);
    // 姿态初始化
    self->ins.filter.xhat_data[0] = 1;
    self->ins.filter.xhat_data[1] = 0;
    self->ins.filter.xhat_data[2] = 0;
    self->ins.filter.xhat_data[3] = 0;
    // 自定义函数初始化,用于扩展或增加kf的基础功能
    self->ins.filter.User_Ptr = self;
    self->ins.filter.User_Func0_f = QuaternionEkf_ObserveCallback;
    self->ins.filter.User_Func1_f = QuaternionEkf_LinearizeCallback;
    self->ins.filter.User_Func2_f = QuaternionEkf_SetHCallback;
    self->ins.filter.User_Func3_f = QuaternionEkf_UpdateXhatCallback;
    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    self->ins.filter.SkipEq3 = true;
    self->ins.filter.SkipEq4 = true;

    memcpy(self->ins.filter.F_data, self->f_matrix, sizeof(self->f_matrix));
    memcpy(self->ins.filter.P_data, self->p_matrix, sizeof(self->p_matrix));
}

/**
 * @brief Quaternion EKF update
 * @param[in] gyro x y z in rad/s
 * @param[in] accel x y z
 * @param[in] update period in s
 */
void QuaternionEkf_Update(QuaternionEkf_t *self, float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    const float acc_norm2 = ax * ax + ay * ay + az * az;
    if (!isfinite(acc_norm2) || acc_norm2 <= 1e-12f || !isfinite(gx) ||
        !isfinite(gy) || !isfinite(gz) || !isfinite(dt) || dt <= 0.0f)
    {
        return;
    }

    // 0.5(Ohm-Ohm^bias)*delta_t,用于更新工作点处的状态转移F矩阵
    static float halfgxdt, halfgydt, halfgzdt;
    static float gyro[3];
    static float accel_inv_norm;
    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    18    19    20    21    22    23
    24    25    26    27    28    29
    30    31    32    33    34    35
    */
    self->ins.dt = dt;

    gyro[0] = gx - self->ins.gyro_bias[0];
    gyro[1] = gy - self->ins.gyro_bias[1];
    gyro[2] = gz - self->ins.gyro_bias[2];

    halfgxdt = 0.5f * (gyro[0]) * dt;
    halfgydt = 0.5f * (gyro[1]) * dt;
    halfgzdt = 0.5f * (gyro[2]) * dt;

    // 初始化F矩阵为单位阵
    memcpy(self->ins.filter.F_data, self->f_matrix, sizeof(self->f_matrix));
    // 此部分设定状态转移矩阵F的左上角部分4x4子矩阵,即0.5(Ohm-Ohm^bias)*delta_t,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    self->ins.filter.F_data[1] = -halfgxdt;
    self->ins.filter.F_data[2] = -halfgydt;
    self->ins.filter.F_data[3] = -halfgzdt;

    self->ins.filter.F_data[6] = halfgxdt;
    self->ins.filter.F_data[8] = halfgzdt;
    self->ins.filter.F_data[9] = -halfgydt;

    self->ins.filter.F_data[12] = halfgydt;
    self->ins.filter.F_data[13] = -halfgzdt;
    self->ins.filter.F_data[15] = halfgxdt;

    self->ins.filter.F_data[18] = halfgzdt;
    self->ins.filter.F_data[19] = halfgydt;
    self->ins.filter.F_data[20] = -halfgxdt;

    // 归一化加速度向量作为量测向量
    const float acc_norm = sqrtf(acc_norm2);
    accel_inv_norm = 1.0f / acc_norm;
    self->ins.filter.MeasuredVector[0] = ax * accel_inv_norm;
    self->ins.filter.MeasuredVector[1] = ay * accel_inv_norm;
    self->ins.filter.MeasuredVector[2] = az * accel_inv_norm;

    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    // 稍后在最后的姿态更新部分会利用stable的flag来确定
    const float gyro_norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    if (gyro_norm < 0.3f && fabsf(acc_norm - 9.8f) < 0.5f)
    {
        self->ins.stable = 1;
    }
    else
    {
        self->ins.stable = 0;
    }

    // 设置Q,R矩阵
    self->ins.filter.Q_data[0] = self->ins.Q1 * self->ins.dt;
    self->ins.filter.Q_data[7] = self->ins.Q1 * self->ins.dt;
    self->ins.filter.Q_data[14] = self->ins.Q1 * self->ins.dt;
    self->ins.filter.Q_data[21] = self->ins.Q1 * self->ins.dt;
    self->ins.filter.Q_data[28] = self->ins.Q2 * self->ins.dt;
    self->ins.filter.Q_data[35] = self->ins.Q2 * self->ins.dt;
    self->ins.filter.R_data[0] = self->ins.R;
    self->ins.filter.R_data[4] = self->ins.R;
    self->ins.filter.R_data[8] = self->ins.R;

    // 卡尔曼滤波器更新
    Kalman_Filter_Update(&self->ins.filter);

    // 估计结果导出
    self->ins.q[0] = self->ins.filter.FilteredValue[0];
    self->ins.q[1] = self->ins.filter.FilteredValue[1];
    self->ins.q[2] = self->ins.filter.FilteredValue[2];
    self->ins.q[3] = self->ins.filter.FilteredValue[3];
    self->ins.gyro_bias[0] = self->ins.filter.FilteredValue[4];
    self->ins.gyro_bias[1] = self->ins.filter.FilteredValue[5];
    self->ins.gyro_bias[2] = -0.0019f;

    // 四元数反解欧拉角
    arm_atan2_f32(2.0f * (self->ins.q[0] * self->ins.q[1] + self->ins.q[2] * self->ins.q[3]),
                  2.0f * (self->ins.q[0] * self->ins.q[0] + self->ins.q[3] * self->ins.q[3]) - 1.0f, &self->ins.angle[0]);
    self->ins.angle[1] = asinf(-2.0f * (self->ins.q[1] * self->ins.q[3] - self->ins.q[0] * self->ins.q[2]));
    arm_atan2_f32(2.0f * (self->ins.q[0] * self->ins.q[3] + self->ins.q[1] * self->ins.q[2]),
                  2.0f * (self->ins.q[0] * self->ins.q[0] + self->ins.q[1] * self->ins.q[1]) - 1.0f, &self->ins.angle[2]);
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 *
 * @param kf
 */
static void QuaternionEkf_Observe(QuaternionEkf_t *self, KalmanFilter_t *kf)
{
    memcpy(self->p_matrix, kf->P_data, sizeof(self->p_matrix));
    memcpy(self->k_matrix, kf->K_data, sizeof(self->k_matrix));
    memcpy(self->h_matrix, kf->H_data, sizeof(self->h_matrix));
}

/**
 * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新;
 *        并对零漂的方差进行限制,防止过度收敛并限幅防止发散
 *
 * @param kf
 */
static void QuaternionEkf_LinearizeFAndFadeP(QuaternionEkf_t *self, KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;
    static float q_inv_norm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // 四元数归一化
    q_inv_norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (int i = 0; i < 4; i++)
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
    kf->F_data[4] = q1 * self->ins.dt / 2;
    kf->F_data[5] = q2 * self->ins.dt / 2;

    kf->F_data[10] = -q0 * self->ins.dt / 2;
    kf->F_data[11] = q3 * self->ins.dt / 2;

    kf->F_data[16] = -q3 * self->ins.dt / 2;
    kf->F_data[17] = -q0 * self->ins.dt / 2;

    kf->F_data[22] = q2 * self->ins.dt / 2;
    kf->F_data[23] = -q1 * self->ins.dt / 2;

    // fading filter,防止零飘参数过度收敛
    kf->P_data[28] /= self->ins.lambda;
    kf->P_data[35] /= self->ins.lambda;

    // 限幅,防止发散
    if (kf->P_data[28] > 10000)
    {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000)
    {
        kf->P_data[35] = 10000;
    }
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 *
 * @param kf
 */
static void QuaternionEkf_SetHMatrix(QuaternionEkf_t *self, KalmanFilter_t *kf)
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

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 *
 * @param kf
 */
static void QuaternionEkf_UpdateXhat(QuaternionEkf_t *self, KalmanFilter_t *kf)
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
    // 计算预测得到的重力加速度方向(通过姿态获取的)
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 计算预测值和各个轴的方向余弦
    for (int i = 0; i < 3; i++)
    {
        self->ins.orientation_arccosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

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
    const float *r = kf->temp_vector1.pData;
    const float *sir = kf->temp_matrix.pData;
    self->ins.chi_square = r[0] * sir[0] + r[1] * sir[1] + r[2] * sir[2];

    // 检测函数
    if (self->ins.chi_square < 0.5f * self->ins.chi_square_threshold)
    {
        self->ins.converged = 1;
    }
    if (self->ins.chi_square > self->ins.chi_square_threshold && self->ins.converged)
    {
        if (self->ins.stable)
        {
            self->ins.error_cnt++;
        }
        else
        {
            self->ins.error_cnt = 0;
        }

        if (self->ins.error_cnt > 50)
        {
            self->ins.converged = 0;
            self->ins.adaptive_gain_scale = 1.0f;
            kf->SkipEq5 = false;
        }
        else
        {
            // 未通过卡方检验 仅预测
            // xhat(k) = xhat'(k)
            // P(k) = P'(k)
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = true;
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk越小则增益越大,否则更相信预测值
        if (self->ins.chi_square > 0.1f * self->ins.chi_square_threshold && self->ins.converged)
        {
            self->ins.adaptive_gain_scale =
                (self->ins.chi_square_threshold - self->ins.chi_square) / (0.9f * self->ins.chi_square_threshold);
        }
        else
        {
            self->ins.adaptive_gain_scale = 1;
        }
        self->ins.error_cnt = 0;
        kf->SkipEq5 = false;
    }

    // 通过卡方检验，进行量测更新
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus =
        Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // implement adaptive
    for (int i = 0; i < kf->K.numRows * kf->K.numCols; i++)
    {
        kf->K_data[i] *= self->ins.adaptive_gain_scale;
    }
    for (int i = 4; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            kf->K_data[i * 3 + j] *= self->ins.orientation_arccosine[i - 4] / (PI / 2);
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1,
                                    &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))

    // 零漂修正限幅,一般不会有过大的漂移
    if (self->ins.converged)
    {
        for (int i = 4; i < 6; i++)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * self->ins.dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * self->ins.dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * self->ins.dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * self->ins.dt;
            }
        }
    }

    kf->temp_vector.pData[3] = 0; // 不修正yaw轴数据
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector,
                               &kf->xhat); // xhat = xhat'(k) + M·K(k)·(z(k) - h(xhat'(k)))
}

static void QuaternionEkf_ObserveCallback(KalmanFilter_t *kf)
{
    QuaternionEkf_t *self = (QuaternionEkf_t *)kf->User_Ptr;
    QuaternionEkf_Observe(self, kf);
}

static void QuaternionEkf_LinearizeCallback(KalmanFilter_t *kf)
{
    QuaternionEkf_t *self = (QuaternionEkf_t *)kf->User_Ptr;
    QuaternionEkf_LinearizeFAndFadeP(self, kf);
}

static void QuaternionEkf_SetHCallback(KalmanFilter_t *kf)
{
    QuaternionEkf_t *self = (QuaternionEkf_t *)kf->User_Ptr;
    QuaternionEkf_SetHMatrix(self, kf);
}

static void QuaternionEkf_UpdateXhatCallback(KalmanFilter_t *kf)
{
    QuaternionEkf_t *self = (QuaternionEkf_t *)kf->User_Ptr;
    QuaternionEkf_UpdateXhat(self, kf);
}
/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
