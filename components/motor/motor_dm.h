/**
 * @file motor_dm.h
 * @author anchengc
 * @brief 达妙电机交互库
 * @version 0.1
 * @date 2026-06-03 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"
#include "math_tools.h"
#include <cstdint>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 达妙电机状态
 *
 */
enum MotorDmStatus
{
    MOTOR_DM_STATUS_DISCONNECTED = 0,
    MOTOR_DM_STATUS_CONNECTED,
};

/**
 * @brief 达妙电机错误码状态
 *
 */
enum MotorDmErrStatus
{
    MOTOR_DM_ERR_STATUS_DISABLE = 0x0,
    MOTOR_DM_ERR_STATUS_ENABLE,
    MOTOR_DM_ERR_STATUS_OVERVOLTAGE = 0x8,
    MOTOR_DM_ERR_STATUS_UNDERVOLTAGE,
    MOTOR_DM_ERR_STATUS_OVERCURRENT,
    MOTOR_DM_ERR_STATUS_MOS_OVERTEMPERATURE,
    MOTOR_DM_ERR_STATUS_ROTOR_OVERTEMPERATURE,
    MOTOR_DM_ERR_STATUS_LOSE_CONNECTION,
    MOTOR_DM_ERR_STATUS_OVERLOAD,
};

/**
 * @brief 达妙电机控制方式
 *
 */
enum MotorDmControlMethod
{
    MOTOR_DM_CONTROL_METHOD_MIT = 0,
    MOTOR_DM_CONTROL_METHOD_ANGLE_OMEGA,
    MOTOR_DM_CONTROL_METHOD_OMEGA,
};

/**
 * @brief 达妙电机反馈的数据
 *
 */
struct MotorDmRxData
{

    MotorDmErrStatus error_status;
    float position;
    float angle;
    float omega;
    float torque;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    float last_position;
    float total_angle;
    int16_t position_cycle_count;
};

/**
 * @brief 达妙电机类
 *
 */
class MotorDm
{
public:
    void init(CAN_HandleTypeDef *hcan, uint8_t can_id, uint8_t master_id,
              MotorDmControlMethod control_method = MOTOR_DM_CONTROL_METHOD_MIT,
              float p_max = 12.5f, float v_max = 30.0f, float t_max = 10.0f, float kp = 0.0f,
              float kd = 0.0f, uint8_t reverse = false);

    inline void set_target_angle(float target_angle);

    inline void set_target_omega(float target_omega);

    inline void set_target_torque(float target_torque);

    inline void set_pid_params(float kp, float kd);

    void can_rx_callback(const uint8_t *rx_data);

    void send_clear_error();

    void send_enable_cmd();

    void send_disable_cmd();

    void save_zero_position();

    void check_alive_100ms();

    void send_control();

    // 电机对外接口信息
    MotorDmRxData rx_data_;

protected:
    // 初始化相关常量

    // 绑定的CAN
    CanManageObject *can_manage_obj_;
    // 发数据绑定的CAN id, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t can_tx_id_;
    // 收数据绑定的CAN id, 与上位机驱动参数Master_ID保持一致
    uint16_t can_rx_id_;
    // 最大位置, 与上位机控制幅值PMAX保持一致
    float p_max_;
    // 最大速度, 与上位机控制幅值VMAX保持一致
    float v_max_;
    // 最大扭矩, 与上位机控制幅值TMAX保持一致
    float t_max_;
    // 反向安装标志
    uint8_t reverse_ = false;

    // 常量

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t rx_flag_ = 0;
    // 前一时刻的电机接收flag
    uint32_t last_rx_flag_ = 0;

    // 输出量
    float cmd_angle_ = 0.0f;
    float cmd_omega_ = 0.0f;
    float cmd_torque_ = 0.0f;
    // 发送缓冲区
    uint8_t tx_data_[8];

    // 读变量

    // 电机状态
    MotorDmStatus status_ = MOTOR_DM_STATUS_DISCONNECTED;

    // 写变量

    // 读写变量

    // 电机控制方式
    MotorDmControlMethod control_method_ = MOTOR_DM_CONTROL_METHOD_MIT;
    // 目标的角度, rad
    float target_angle_ = 0.0f;
    // 目标的速度, rad/s
    float target_omega_ = 0.0f;
    // 目标的扭矩, N*m
    float target_torque_ = 0.0f;
    // kp_, 0~500
    float kp_ = 0.0f;
    // kd_, 0~5
    float kd_ = 0.0f;

    // 内部函数

    void process_data(const uint8_t *rx_data);

    void output();

    void send_mit_control();

    void send_angle_omega_control();

    void send_omega_control();
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 设定目标的角度, rad
 *
 * @param target_angle 目标的角度, rad
 */
inline void MotorDm::set_target_angle(float target_angle)
{
    target_angle_ = target_angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param target_omega 目标的速度, rad/s
 */
inline void MotorDm::set_target_omega(float target_omega)
{
    target_omega_ = target_omega;
}

/**
 * @brief 设定目标的扭矩, N*m
 *
 * @param target_torque 目标的扭矩, N*m
 */
inline void MotorDm::set_target_torque(float target_torque)
{
    target_torque_ = target_torque;
}

/**
 * @brief 设定PID参数
 *
 * @param kp kp, 0~500
 * @param kd kd, 0~5
 */
inline void MotorDm::set_pid_params(float kp, float kd)
{
    kp_ = kp;
    kd_ = kd;
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
