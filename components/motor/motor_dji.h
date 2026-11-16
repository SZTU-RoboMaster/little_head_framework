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

#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 大疆电机状态
 */
typedef enum
{
    MOTOR_DJI_STATUS_DISABLE = 0,
    MOTOR_DJI_STATUS_ENABLE,
} MotorDjiStatus_e;

/**
 * @brief 大疆电机控制方式
 */
typedef enum
{
    MOTOR_DJI_CONTROL_METHOD_CURRENT = 0,
    MOTOR_DJI_CONTROL_METHOD_OMEGA,
    MOTOR_DJI_CONTROL_METHOD_ANGLE,
} MotorDjiControlMethod_e;

/**
 * @brief 大疆电机反馈的数据
 */
typedef struct
{
    uint16_t encoder;
    float angle;
    float omega;
    int16_t current;
    uint8_t temperature;
    uint16_t last_encoder;
    float total_angle;
    int16_t round_count;
} MotorDjiRxData_t;

/**
 * @brief 大疆电机对象，保存控制参数、反馈和 CAN 发送位置
 */
typedef struct
{
    // PID角度环控制
    Pid_t angle_pid;
    // PID角速度环控制
    Pid_t omega_pid;
    // 电机对外接口信息，由 CAN 接收回调更新
    MotorDjiRxData_t rx_data;

    // 绑定的CAN
    CAN_HandleTypeDef *can_handle;
    // 发数据绑定的CAN id, C6系列0x200/0x1FF, GM系列0x1FE/0x2FE
    uint16_t can_tx_id;
    // 收数据绑定的CAN id, C6系列0x201~0x208, GM系列0x205~0x20b
    uint16_t can_rx_id;
    // 发送缓存区中的两个字节位置
    uint8_t *tx_data;
    // 减速比, 默认带减速箱
    float gearbox_rate;
    // 反向安装标志
    uint8_t reverse;

    // 当前时刻的电机接收flag
    volatile uint32_t rx_flag;
    // 前一时刻的电机接收flag
    uint32_t last_rx_flag;
    // 计算周期计数, 用于降低角度环计算频率
    uint8_t calculate_mod5;

    // 当前时刻的电机控制flag
    volatile uint32_t cmd_flag;
    // 前一时刻的电机控制flag
    uint32_t last_cmd_flag;
    // 控制指令是否在线
    uint8_t cmd_online;

    // 输出量
    float output_value;
    // 电机状态
    MotorDjiStatus_e status;
    // 电机控制方式
    MotorDjiControlMethod_e control_method;
    // 目标的角度, rad
    float target_angle;
    // 目标的速度, rad/s
    float target_omega;
    // 目标的电流, 控制帧原始值
    float target_current;
    // 前馈的速度, rad/s
    float feedforward_omega;
} MotorDji_t;

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t can1_0x200_tx_data[8];
extern uint8_t can1_0x1ff_tx_data[8];
extern uint8_t can1_0x1fe_tx_data[8];
extern uint8_t can1_0x2fe_tx_data[8];
extern uint8_t can2_0x200_tx_data[8];
extern uint8_t can2_0x1ff_tx_data[8];
extern uint8_t can2_0x1fe_tx_data[8];
extern uint8_t can2_0x2fe_tx_data[8];

bool MotorDji_Init(MotorDji_t *motor, CAN_HandleTypeDef *hcan, uint16_t can_tx_id,
                   uint16_t can_rx_id, MotorDjiControlMethod_e control_method,
                   float gearbox_rate, uint8_t reverse);

void MotorDji_SetTargetCurrent(MotorDji_t *motor, float target_current);
void MotorDji_SetTargetAngle(MotorDji_t *motor, float target_angle);
void MotorDji_SetTargetOmega(MotorDji_t *motor, float target_omega);
void MotorDji_SetFeedforwardOmega(MotorDji_t *motor, float feedforward_omega);
void MotorDji_SetControlMethod(MotorDji_t *motor, MotorDjiControlMethod_e control_method);

void MotorDji_CanRxCallback(MotorDji_t *motor, const uint8_t *rx_data);
void MotorDji_CheckAlive100ms(MotorDji_t *motor);
void MotorDji_Calculate(MotorDji_t *motor);

#ifdef __cplusplus
}
#endif

#endif
