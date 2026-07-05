/**
 * @file motor_dm.cpp
 * @author anchengc
 * @brief 达妙电机交互库
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "motor_dm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 清除错误信息帧
uint8_t dm_motor_clear_error_msg[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};

// 使能帧
uint8_t dm_motor_enable_msg[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};

// 失能帧
uint8_t dm_motor_disable_msg[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};

// 保存零点帧
uint8_t dm_motor_save_zero_msg[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param can_id 发数据绑定的CAN id, 是上位机驱动参数CAN_ID
 * @param master_id 收数据绑定的CAN id, 与上位机驱动参数Master_ID保持一致
 * @param control_method 电机控制方式
 * @param p_max 最大位置, 与上位机控制幅值PMAX保持一致
 * @param v_max 最大速度, 与上位机控制幅值VMAX保持一致
 * @param t_max 最大扭矩, 与上位机控制幅值TMAX保持一致
 * @param kp MIT控制的K_P
 * @param kd MIT控制的K_D
 * @param __I_Max 最大电流, 与上位机串口中上电打印Imax保持一致
 */
void MotorDm::init(CAN_HandleTypeDef *hcan, uint8_t can_id, uint8_t master_id,
                   MotorDmControlMethod control_method, float p_max, float v_max, float t_max,
                   float kp, float kd, uint8_t reverse)
{
    if (hcan->Instance == CAN1)
    {
        can_manage_obj_ = &can1_manage_obj;
    }
    else if (hcan->Instance == CAN2)
    {
        can_manage_obj_ = &can2_manage_obj;
    }

    can_rx_id_ = master_id;
    switch (control_method)
    {
    case (MOTOR_DM_CONTROL_METHOD_MIT):
    {
        can_tx_id_ = can_id;
        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_ANGLE_OMEGA):
    {
        can_tx_id_ = can_id + 0x100;
        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_OMEGA):
    {
        can_tx_id_ = can_id + 0x200;
        break;
    }
    }
    control_method_ = control_method;
    p_max_ = p_max;
    v_max_ = v_max;
    t_max_ = t_max;
    kp_ = kp;
    kd_ = kd;
    reverse_ = reverse;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void MotorDm::can_rx_callback(const uint8_t *rx_data)
{
    // 滑动窗口, 判断电机是否在线
    rx_flag_ += 1;

    process_data(rx_data);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void MotorDm::check_alive_100ms()
{
    // 判断该时间段内是否接收过电机数据
    if (rx_flag_ == last_rx_flag_)
    {
        // 电机断开连接
        status_ = MOTOR_DM_STATUS_DISCONNECTED;

        // 发送使能帧尝试重新连接
        send_enable_cmd();
    }
    else
    {
        // 电机保持连接
        status_ = MOTOR_DM_STATUS_CONNECTED;
    }

    last_rx_flag_ = rx_flag_;
}

/**
 * @brief TIM定时器中断发送出去的回调函数, 计算周期取决于自主设置的控制周期
 *
 */
void MotorDm::send_control()
{
    if (status_ == MOTOR_DM_STATUS_CONNECTED)
    {
        if (rx_data_.error_status == MOTOR_DM_ERR_STATUS_ENABLE)
        {
            // 电机在线, 正常控制
            cmd_angle_ = reverse_ ? -target_angle_ : target_angle_;
            cmd_omega_ = reverse_ ? -target_omega_ : target_omega_;
            cmd_torque_ = reverse_ ? -target_torque_ : target_torque_;
            math_constrain(&cmd_angle_, -p_max_, p_max_);
            math_constrain(&cmd_omega_, -v_max_, v_max_);
            math_constrain(&cmd_torque_, -t_max_, t_max_);
            math_constrain(&kp_, 0.0f, 500.0f);
            math_constrain(&kd_, 0.0f, 5.0f);

            output();
        }
        else if (rx_data_.error_status == MOTOR_DM_ERR_STATUS_DISABLE)
        {
            // 电机失能, 发送使能帧
            send_enable_cmd();
        }
        else
        {
            // 电机异常, 发送清除错误信息帧
            send_clear_error();
        }
    }
    else
    {
        return;
    }
}

/**
 * @brief 数据处理过程
 *
 */
void MotorDm::process_data(const uint8_t *rx_data)
{
    // 数据处理过程

    // 电机ID不匹配, 则不进行处理
    if ((rx_data[0] & 0x0f) != (can_tx_id_ & 0x0f))
    {
        return;
    }

    float delta_position;

    // 计算电机本身信息
    rx_data_.error_status = static_cast<MotorDmErrStatus>(rx_data[0] >> 4);
    rx_data_.position = uint_to_float(rx_data[1] << 8 | rx_data[2], -p_max_, p_max_, 16);
    rx_data_.omega = uint_to_float(rx_data[3] << 4 | rx_data[4] >> 4, -v_max_, v_max_, 12);
    rx_data_.torque = uint_to_float((rx_data[4] & 0x0f) << 8 | rx_data[5], -t_max_, t_max_, 12);
    rx_data_.mos_temperature = rx_data[6];
    rx_data_.rotor_temperature = rx_data[7];

    // 反向安装处理
    if (reverse_ == true)
    {
        rx_data_.position = -rx_data_.position;
        rx_data_.omega = -rx_data_.omega;
        rx_data_.torque = -rx_data_.torque;
    }

    // 初始化预备位置值
    if (rx_flag_ == 1)
    {
        rx_data_.last_position = rx_data_.position;
    }

    // 计算圈数与总位置值
    delta_position = rx_data_.position - rx_data_.last_position;
    if (delta_position < -p_max_)
    {
        // 正方向转过了一个P_MAX周期
        rx_data_.position_cycle_count++;
    }
    else if (delta_position > p_max_)
    {
        // 反方向转过了一个P_MAX周期
        rx_data_.position_cycle_count--;
    }
    rx_data_.total_position = rx_data_.position_cycle_count * 2.0f * p_max_ + rx_data_.position;

    // 计算角度, 归化到 -PI ~ PI
    rx_data_.angle = math_modulus_normalize((double)rx_data_.total_position, 2.0f * M_PI);

    // 存储预备信息
    rx_data_.last_position = rx_data_.position;
}

/**
 * @brief 电机数据输出到CAN总线
 *
 */
void MotorDm::output()
{
    switch (control_method_)
    {
    case (MOTOR_DM_CONTROL_METHOD_MIT):
    {
        send_mit_control();
        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_ANGLE_OMEGA):
    {
        send_angle_omega_control();
        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_OMEGA):
    {
        send_omega_control();
        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @brief 发送清除错误信息帧
 *
 */
void MotorDm::send_clear_error(void)
{
    can_data_send(can_manage_obj_->can_handle, can_tx_id_, dm_motor_clear_error_msg, 8);
}

/**
 * @brief 发送使能帧
 *
 */
void MotorDm::send_enable_cmd(void)
{
    can_data_send(can_manage_obj_->can_handle, can_tx_id_, dm_motor_enable_msg, 8);
}

/**
 * @brief 发送失能帧
 *
 */
void MotorDm::send_disable_cmd(void)
{
    can_data_send(can_manage_obj_->can_handle, can_tx_id_, dm_motor_disable_msg, 8);
}

/**
 * @brief 发送保存零点帧
 *
 */
void MotorDm::save_zero_position(void)
{
    can_data_send(can_manage_obj_->can_handle, can_tx_id_, dm_motor_save_zero_msg, 8);
}

/**
 * @brief 发送MIT控制帧
 *
 */
void MotorDm::send_mit_control()
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(cmd_angle_, -p_max_, p_max_, 16);
    vel_tmp = float_to_uint(cmd_omega_, -v_max_, v_max_, 12);
    kp_tmp = float_to_uint(kp_, 0.0f, 500.0f, 12);
    kd_tmp = float_to_uint(kd_, 0.0f, 5.0f, 12);
    tor_tmp = float_to_uint(cmd_torque_, -t_max_, t_max_, 12);

    tx_data_[0] = (pos_tmp >> 8);
    tx_data_[1] = pos_tmp;
    tx_data_[2] = (vel_tmp >> 4);
    tx_data_[3] = (vel_tmp & 0xf) << 4 | (kp_tmp >> 8);
    tx_data_[4] = kp_tmp;
    tx_data_[5] = kd_tmp >> 4;
    tx_data_[6] = (kd_tmp & 0xf) << 4 | (tor_tmp >> 8);
    tx_data_[7] = tor_tmp;

    can_data_send(can_manage_obj_->can_handle, can_tx_id_, tx_data_, 8);
}

/**
 * @brief 发送位置速度控制帧
 *
 */
void MotorDm::send_angle_omega_control()
{
    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&cmd_angle_;
    vbuf = (uint8_t *)&cmd_omega_;

    tx_data_[0] = *pbuf;
    tx_data_[1] = *(pbuf + 1);
    tx_data_[2] = *(pbuf + 2);
    tx_data_[3] = *(pbuf + 3);
    tx_data_[4] = *vbuf;
    tx_data_[5] = *(vbuf + 1);
    tx_data_[6] = *(vbuf + 2);
    tx_data_[7] = *(vbuf + 3);

    can_data_send(can_manage_obj_->can_handle, can_tx_id_, tx_data_, 8);
}

/**
 * @brief 发送速度控制帧
 *
 */
void MotorDm::send_omega_control()
{
    uint8_t *vbuf;
    vbuf = (uint8_t *)&cmd_omega_;

    tx_data_[0] = *vbuf;
    tx_data_[1] = *(vbuf + 1);
    tx_data_[2] = *(vbuf + 2);
    tx_data_[3] = *(vbuf + 3);
    tx_data_[4] = 0;
    tx_data_[5] = 0;
    tx_data_[6] = 0;
    tx_data_[7] = 0;

    can_data_send(can_manage_obj_->can_handle, can_tx_id_, tx_data_, 8);
}
/************************ COPYRIGHT(C) SZTU-HJ **************************/
