/**
 * @file motor_dji.cpp
 * @author anchengc
 * @brief 大疆电机交互库
 * @version 0.1
 * @date 2026-06-05 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "motor_dji.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// CAN通信发送缓冲区

uint8_t can1_0x200_tx_data[8];
uint8_t can1_0x1fe_tx_data[8];
uint8_t can1_0x2fe_tx_data[8];
uint8_t can2_0x200_tx_data[8];
uint8_t can2_0x1fe_tx_data[8];
uint8_t can2_0x2fe_tx_data[8];

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param can_id CAN id
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, uint16_t can_id)
{
    uint8_t *tmp_tx_data_ptr = nullptr;
    if (hcan == &hcan1)
    {
        switch (can_id)
        {
        case (0x201):
        {
            tmp_tx_data_ptr = &(can1_0x200_tx_data[0]);

            break;
        }
        case (0x202):
        {
            tmp_tx_data_ptr = &(can1_0x200_tx_data[2]);

            break;
        }
        case (0x203):
        {
            tmp_tx_data_ptr = &(can1_0x200_tx_data[4]);

            break;
        }
        case (0x204):
        {
            tmp_tx_data_ptr = &(can1_0x200_tx_data[6]);

            break;
        }
        case (0x205):
        {
            tmp_tx_data_ptr = &(can1_0x1fe_tx_data[0]);

            break;
        }
        case (0x206):
        {
            tmp_tx_data_ptr = &(can1_0x1fe_tx_data[2]);

            break;
        }
        case (0x207):
        {
            tmp_tx_data_ptr = &(can1_0x1fe_tx_data[4]);

            break;
        }
        case (0x208):
        {
            tmp_tx_data_ptr = &(can1_0x1fe_tx_data[6]);

            break;
        }
        case (0x209):
        {
            tmp_tx_data_ptr = &(can1_0x2fe_tx_data[0]);

            break;
        }
        case (0x20a):
        {
            tmp_tx_data_ptr = &(can1_0x2fe_tx_data[2]);

            break;
        }
        case (0x20b):
        {
            tmp_tx_data_ptr = &(can1_0x2fe_tx_data[4]);

            break;
        }
        }
    }
    else if (hcan == &hcan2)
    {
        switch (can_id)
        {
        case (0x201):
        {
            tmp_tx_data_ptr = &(can2_0x200_tx_data[0]);

            break;
        }
        case (0x202):
        {
            tmp_tx_data_ptr = &(can2_0x200_tx_data[2]);

            break;
        }
        case (0x203):
        {
            tmp_tx_data_ptr = &(can2_0x200_tx_data[4]);

            break;
        }
        case (0x204):
        {
            tmp_tx_data_ptr = &(can2_0x200_tx_data[6]);

            break;
        }
        case (0x205):
        {
            tmp_tx_data_ptr = &(can2_0x1fe_tx_data[0]);

            break;
        }
        case (0x206):
        {
            tmp_tx_data_ptr = &(can2_0x1fe_tx_data[2]);

            break;
        }
        case (0x207):
        {
            tmp_tx_data_ptr = &(can2_0x1fe_tx_data[4]);

            break;
        }
        case (0x208):
        {
            tmp_tx_data_ptr = &(can2_0x1fe_tx_data[6]);

            break;
        }
        case (0x209):
        {
            tmp_tx_data_ptr = &(can2_0x2fe_tx_data[0]);

            break;
        }
        case (0x20a):
        {
            tmp_tx_data_ptr = &(can2_0x2fe_tx_data[2]);

            break;
        }
        case (0x20b):
        {
            tmp_tx_data_ptr = &(can2_0x2fe_tx_data[4]);

            break;
        }
        }
    }
    return (tmp_tx_data_ptr);
}

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param can_rx_id 绑定的CAN id
 * @param control_method 控制方法
 * @param gearbox_rate 减速比
 */
void MotorDji::init(CAN_HandleTypeDef *hcan, uint16_t can_rx_id,
                    MotorDjiControlMethod control_method, float gearbox_rate, uint8_t reverse)
{
    if (hcan->Instance == CAN1)
    {
        can_manage_obj_ = &can1_manage_obj;
    }
    else if (hcan->Instance == CAN2)
    {
        can_manage_obj_ = &can2_manage_obj;
    }
    can_rx_id_ = can_rx_id;
    control_method_ = control_method;
    gearbox_rate_ = gearbox_rate;
    reverse_ = reverse;
    tx_data_ = allocate_tx_data(hcan, can_rx_id);
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void MotorDji::can_rx_callback(const uint8_t *rx_data)
{
    // 滑动窗口, 判断电机是否在线
    rx_flag_ += 1;

    process_data(rx_data);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void MotorDji::check_alive_100ms()
{
    // 判断该时间段内是否接收过电机数据
    if (rx_flag_ == last_rx_flag_)
    {
        // 电机断开连接
        status_ = MOTOR_DJI_STATUS_DISABLE;
        angle_pid_.set_integral_error(0.0f);
        omega_pid_.set_integral_error(0.0f);
    }
    else
    {
        // 电机保持连接
        status_ = MOTOR_DJI_STATUS_ENABLE;
    }
    last_rx_flag_ = rx_flag_;
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void MotorDji::calculate()
{
    if (status_ == MOTOR_DJI_STATUS_ENABLE)
    {
        calculate_control();
        output_value_ = reverse_ ? -target_current_ : target_current_;
        output();
    }
    else
    {
        output_value_ = 0.0f;
        output();
    }
}

/**
 * @brief 数据处理过程
 *
 */
void MotorDji::process_data(const uint8_t *rx_data)
{
    // 数据处理过程

    int16_t delta_encoder;

    // 计算电机本身信息
    rx_data_.encoder = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
    rx_data_.omega = (int16_t)(rx_data[2] << 8 | rx_data[3]) * RPM_TO_RADPS / gearbox_rate_;
    rx_data_.current = (int16_t)(rx_data[4] << 8 | rx_data[5]);
    rx_data_.temperature = rx_data[6];

    // 反向安装处理
    if (reverse_ == true)
    {
        rx_data_.encoder = (8192 - rx_data_.encoder) % 8192;
        rx_data_.omega = -rx_data_.omega;
        rx_data_.current = -rx_data_.current;
    }

    // 初始化预备编码器值
    if (rx_flag_ == 1)
    {
        rx_data_.last_encoder = rx_data_.encoder;
    }

    // 计算圈数与总编码器值
    delta_encoder = rx_data_.encoder - rx_data_.last_encoder;
    if (delta_encoder < -8192 / 2)
    {
        // 正方向转过了一圈
        rx_data_.round_count++;
    }
    else if (delta_encoder > 8192 / 2)
    {
        // 反方向转过了一圈
        rx_data_.round_count--;
    }
    rx_data_.total_encoder = rx_data_.round_count * 8192 + rx_data_.encoder;

    // 计算角度
    rx_data_.angle = (float)rx_data_.total_encoder / (float)8192 * 2.0f * M_PI / gearbox_rate_;

    // 存储预备信息
    rx_data_.last_encoder = rx_data_.encoder;
}

/**
 * @brief 计算PID
 *
 */
void MotorDji::calculate_control()
{
    switch (control_method_)
    {
    case (MOTOR_DJI_CONTROL_METHOD_CURRENT):
    {
        break;
    }
    case (MOTOR_DJI_CONTROL_METHOD_OMEGA):
    {

        omega_pid_.set_target(target_omega_ + feedforward_omega_);
        omega_pid_.set_feedback(rx_data_.omega);
        omega_pid_.calculate();

        target_current_ = omega_pid_.get_output();

        break;
    }
    case (MOTOR_DJI_CONTROL_METHOD_ANGLE):
    {
        calculate_mod5_++;
        if (calculate_mod5_ >= 5)
        {
            calculate_mod5_ = 0;

            angle_pid_.set_target(target_angle_);
            angle_pid_.set_feedback(rx_data_.angle);
            angle_pid_.calculate();
        }

        target_omega_ = angle_pid_.get_output();

        omega_pid_.set_target(target_omega_ + feedforward_omega_);
        omega_pid_.set_feedback(rx_data_.omega);
        omega_pid_.calculate();

        target_current_ = omega_pid_.get_output();

        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 * @param
 */
void MotorDji::output()
{
    // 填充CAN发送数据
    tx_data_[0] = (int16_t)output_value_ >> 8;
    tx_data_[1] = (int16_t)output_value_;
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
