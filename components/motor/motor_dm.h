/**
 * @file motor_dm.h
 * @author anchengc
 * @brief 达妙电机交互库
 * @version 0.1
 * @date 2026-06-03 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

#ifndef MOTOR_DM_H
#define MOTOR_DM_H

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 达妙电机状态
 */
typedef enum
{
    MOTOR_DM_STATUS_DISCONNECTED = 0,
    MOTOR_DM_STATUS_CONNECTED,
} MotorDmStatus_e;

/**
 * @brief 达妙电机错误码状态
 */
typedef enum
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
} MotorDmErrStatus_e;

/**
 * @brief 达妙电机控制方式
 */
typedef enum
{
    MOTOR_DM_CONTROL_METHOD_MIT = 0,
    MOTOR_DM_CONTROL_METHOD_ANGLE_OMEGA,
    MOTOR_DM_CONTROL_METHOD_OMEGA,
} MotorDmControlMethod_e;

/**
 * @brief 达妙电机使能状态
 */
typedef enum
{
    MOTOR_DM_ENABLE_STATUS_DISABLE = 0,
    MOTOR_DM_ENABLE_STATUS_ENABLE,
} MotorDmEnableStatus_e;

/**
 * @brief 达妙电机反馈的数据
 */
typedef struct
{
    MotorDmErrStatus_e error_status;
    float position;
    float angle;
    float omega;
    float torque;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    float last_position;
    float total_angle;
    int16_t position_cycle_count;
} MotorDmRxData_t;

/**
 * @brief 达妙电机对象，保存控制参数、反馈和 CAN 发送缓存
 */
typedef struct
{
    // 电机对外接口信息
    MotorDmRxData_t rx_data;

    // 绑定的CAN
    BspCan_Manage_t *can_manage_obj;
    // 发数据绑定的CAN id, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t can_tx_id;
    // 收数据绑定的CAN id, 与上位机驱动参数Master_ID保持一致
    uint16_t can_rx_id;
    // 最大位置, 与上位机控制幅值PMAX保持一致
    float p_max;
    // 最大速度, 与上位机控制幅值VMAX保持一致
    float v_max;
    // 最大扭矩, 与上位机控制幅值TMAX保持一致
    float t_max;
    // 反向安装标志
    uint8_t reverse;

    // 当前时刻的电机接收flag
    volatile uint32_t rx_flag;
    // 前一时刻的电机接收flag
    uint32_t last_rx_flag;
    // 当前时刻的电机控制flag
    volatile uint32_t cmd_flag;
    // 前一时刻的电机控制flag
    uint32_t last_cmd_flag;
    // 控制指令是否在线
    uint8_t cmd_online;

    // 输出量
    float cmd_angle;
    float cmd_omega;
    float cmd_torque;
    // 发送缓冲区
    uint8_t tx_data[8];

    // 电机状态
    MotorDmStatus_e status;
    MotorDmEnableStatus_e enable_status;
    // 电机控制方式
    MotorDmControlMethod_e control_method;
    // 目标的角度, rad
    float target_angle;
    // 目标的速度, rad/s
    float target_omega;
    // 目标的扭矩, N*m
    float target_torque;
    // kp, 0~500
    float kp;
    // kd, 0~5
    float kd;
} MotorDm_t;

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void MotorDm_Init(MotorDm_t *motor, CAN_HandleTypeDef *hcan, uint16_t can_id, uint16_t master_id,
                  MotorDmControlMethod_e control_method, float p_max, float v_max, float t_max,
                  float kp, float kd, uint8_t reverse, MotorDmEnableStatus_e enable_status);

void MotorDm_SetTargetAngle(MotorDm_t *motor, float target_angle);
void MotorDm_SetTargetOmega(MotorDm_t *motor, float target_omega);
void MotorDm_SetTargetTorque(MotorDm_t *motor, float target_torque);
void MotorDm_SetPidParams(MotorDm_t *motor, float kp, float kd);
void MotorDm_SetEnableStatus(MotorDm_t *motor, MotorDmEnableStatus_e enable_status);

void MotorDm_CanRxCallback(MotorDm_t *motor, const uint8_t *rx_data);
void MotorDm_CheckAlive100ms(MotorDm_t *motor);
void MotorDm_SendControl(MotorDm_t *motor);
void MotorDm_SendClearError(MotorDm_t *motor);
void MotorDm_SendEnableCmd(MotorDm_t *motor);
void MotorDm_SendDisableCmd(MotorDm_t *motor);
void MotorDm_SaveZeroPosition(MotorDm_t *motor);

#ifdef __cplusplus
}
#endif

#endif
