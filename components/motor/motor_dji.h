/**
 * @file motor_dji.h
 * @author anchengc
 * @brief 大疆电机交互库
 * @version 0.1
 * @date 2026-06-05 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"
#include "pid.h"
#include <cstdint>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 大疆电机状态
 *
 */
enum MotorDjiStatus
{
    MOTOR_DJI_STATUS_DISABLE = 0,
    MOTOR_DJI_STATUS_ENABLE,
};

/**
 * @brief 大疆电机控制方式
 *
 */
enum MotorDjiControlMethod
{
    MOTOR_DJI_CONTROL_METHOD_CURRENT = 0,
    MOTOR_DJI_CONTROL_METHOD_OMEGA,
    MOTOR_DJI_CONTROL_METHOD_ANGLE,
};

/**
 * @brief 大疆电机反馈的数据
 *
 */
struct MotorDjiRxData
{
    uint16_t encoder;
    float angle;
    float omega;
    int16_t current;
    uint8_t temperature;
    uint16_t last_encoder;
    float total_angle;
    int16_t round_count;
};

/**
 * @brief 大疆电机类
 *
 */
class MotorDji
{
public:
    // PID角度环控制
    Pid angle_pid_;
    // PID角速度环控制
    Pid omega_pid_;
    // 电机对外接口信息
    MotorDjiRxData rx_data_;

    void init(CAN_HandleTypeDef *hcan, uint16_t can_tx_id, uint16_t can_rx_id,
              MotorDjiControlMethod control_method = MOTOR_DJI_CONTROL_METHOD_OMEGA,
              float gearbox_rate = 1.0f, uint8_t reverse = false);

    inline void set_target_current(float target_current);

    inline void set_target_angle(float target_angle);

    inline void set_target_omega(float target_omega);

    inline void set_feedforward_omega(float feedforward_omega);

    inline void set_control_method(MotorDjiControlMethod control_method);

    void can_rx_callback(const uint8_t *rx_data);

    void check_alive_100ms();

    void calculate();

protected:
    // 初始化相关常量

    // 绑定的CAN
    CanManageObject *can_manage_obj_;
    // 发数据绑定的CAN id, C6系列0x200/0x1FF, GM系列0x1FE/0x2FE
    uint16_t can_tx_id_;
    // 收数据绑定的CAN id, C6系列0x201~0x208, GM系列0x205~0x20b
    uint16_t can_rx_id_;
    // 发送缓存区
    uint8_t *tx_data_;
    // 减速比, 默认带减速箱
    float gearbox_rate_ = 1.0f;
    // 反向安装标志
    uint8_t reverse_ = false;

    // 常量

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t rx_flag_ = 0;
    // 前一时刻的电机接收flag
    uint32_t last_rx_flag_ = 0;
    // 计算周期计数, 用于降低角度环计算频率
    uint8_t calculate_mod5_ = 5;

    // 输出量
    float output_value_ = 0.0f;

    // 读变量

    // 电机状态
    MotorDjiStatus status_ = MOTOR_DJI_STATUS_DISABLE;

    // 写变量

    // 读写变量

    // 电机控制方式
    MotorDjiControlMethod control_method_ = MOTOR_DJI_CONTROL_METHOD_CURRENT;
    // 目标的角度, rad
    float target_angle_ = 0.0f;
    // 目标的速度, rad/s
    float target_omega_ = 0.0f;
    // 目标的电流, A
    float target_current_ = 0.0f;
    // 前馈的速度, rad/s
    float feedforward_omega_ = 0.0f;

    // 内部函数

    void process_data(const uint8_t *rx_data);

    void calculate_control();

    void output();
};

/* Exported variables ---------------------------------------------------------*/

extern uint8_t can1_0x200_tx_data[];
extern uint8_t can1_0x1ff_tx_data[];
extern uint8_t can1_0x1fe_tx_data[];
extern uint8_t can1_0x2fe_tx_data[];
extern uint8_t can2_0x200_tx_data[];
extern uint8_t can2_0x1ff_tx_data[];
extern uint8_t can2_0x1fe_tx_data[];
extern uint8_t can2_0x2fe_tx_data[];

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 设定目标的电流, A
 *
 * @param target_current 目标的电流, A
 */
inline void MotorDji::set_target_current(float target_current)
{
    target_current_ = target_current;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param target_angle 目标的角度, rad
 */
inline void MotorDji::set_target_angle(float target_angle)
{
    target_angle_ = target_angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param target_omega 目标的速度, rad/s
 */
inline void MotorDji::set_target_omega(float target_omega)
{
    target_omega_ = target_omega;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param feedforward_omega 前馈的速度, rad/s
 */
inline void MotorDji::set_feedforward_omega(float feedforward_omega)
{
    feedforward_omega_ = feedforward_omega;
}

/**
 * @brief 设定电机控制方式
 *
 * @param control_method 电机控制方式
 */
inline void MotorDji::set_control_method(MotorDjiControlMethod control_method)
{
    control_method_ = control_method;
}