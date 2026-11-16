/**
 * @file motor_dji.c
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

#include "math_tools.h"
#include <stddef.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// CAN通信发送缓冲区
uint8_t can1_0x200_tx_data[8];
uint8_t can1_0x1ff_tx_data[8];
uint8_t can1_0x1fe_tx_data[8];
uint8_t can1_0x2fe_tx_data[8];
uint8_t can2_0x200_tx_data[8];
uint8_t can2_0x1ff_tx_data[8];
uint8_t can2_0x1fe_tx_data[8];
uint8_t can2_0x2fe_tx_data[8];

/* Private function declarations ---------------------------------------------*/
static uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, uint16_t can_tx_id,
                                 uint16_t can_rx_id);
static void MotorDji_ProcessData(MotorDji_t *motor, const uint8_t *rx_data);
static void MotorDji_CalculateControl(MotorDji_t *motor);
static void MotorDji_Output(MotorDji_t *motor);

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param can_tx_id 发送帧ID
 * @param can_rx_id 电机反馈ID
 * @return uint8_t* 两字节发送槽位，参数错误返回NULL
 */
static uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, uint16_t can_tx_id,
                                 uint16_t can_rx_id)
{
    uint8_t *data = NULL;
    uint16_t first_rx_id = 0;

    if (hcan == NULL)
    {
        return NULL;
    }

    if (hcan->Instance == CAN1)
    {
        switch (can_tx_id)
        {
        case 0x200: data = can1_0x200_tx_data; first_rx_id = 0x201; break;
        case 0x1ff: data = can1_0x1ff_tx_data; first_rx_id = 0x205; break;
        case 0x1fe: data = can1_0x1fe_tx_data; first_rx_id = 0x205; break;
        case 0x2fe: data = can1_0x2fe_tx_data; first_rx_id = 0x209; break;
        default: return NULL;
        }
    }
    else if (hcan->Instance == CAN2)
    {
        switch (can_tx_id)
        {
        case 0x200: data = can2_0x200_tx_data; first_rx_id = 0x201; break;
        case 0x1ff: data = can2_0x1ff_tx_data; first_rx_id = 0x205; break;
        case 0x1fe: data = can2_0x1fe_tx_data; first_rx_id = 0x205; break;
        case 0x2fe: data = can2_0x2fe_tx_data; first_rx_id = 0x209; break;
        default: return NULL;
        }
    }

    // 聚合帧只有四个电机槽位，必须先检查范围再计算偏移
    if (data == NULL || can_rx_id < first_rx_id || can_rx_id >= first_rx_id + 4)
    {
        return NULL;
    }
    return &data[(can_rx_id - first_rx_id) * 2];
}

/**
 * @brief 电机初始化
 *
 * @param motor 电机对象
 * @param hcan 绑定的CAN总线
 * @param can_tx_id 聚合帧发送ID
 * @param can_rx_id 绑定的电机反馈ID
 * @param control_method 控制方法
 * @param gearbox_rate 减速比
 * @param reverse 反向安装标志
 * @return bool 初始化成功返回true
 */
bool MotorDji_Init(MotorDji_t *motor, CAN_HandleTypeDef *hcan, uint16_t can_tx_id,
                   uint16_t can_rx_id, MotorDjiControlMethod_e control_method,
                   float gearbox_rate, uint8_t reverse)
{
    uint8_t *tx_data = allocate_tx_data(hcan, can_tx_id, can_rx_id);
    if (motor == NULL || tx_data == NULL || gearbox_rate <= 0.0f ||
        control_method > MOTOR_DJI_CONTROL_METHOD_ANGLE)
    {
        return false;
    }

    // PID可在电机初始化前由应用配置，这里不清除两个PID对象
    motor->can_handle = hcan;
    motor->can_tx_id = can_tx_id;
    motor->can_rx_id = can_rx_id;
    motor->control_method = control_method;
    motor->gearbox_rate = gearbox_rate;
    motor->reverse = reverse;
    motor->tx_data = tx_data;
    motor->rx_data = (MotorDjiRxData_t){0};
    motor->rx_flag = 0;
    motor->last_rx_flag = 0;
    motor->calculate_mod5 = 5;
    motor->cmd_flag = 0;
    motor->last_cmd_flag = 0;
    motor->cmd_online = 0;
    motor->output_value = 0.0f;
    motor->status = MOTOR_DJI_STATUS_DISABLE;
    motor->target_angle = 0.0f;
    motor->target_omega = 0.0f;
    motor->target_current = 0.0f;
    motor->feedforward_omega = 0.0f;
    tx_data[0] = 0;
    tx_data[1] = 0;
    return true;
}

/**
 * @brief 设定目标的电流, 控制帧原始值
 *
 * @param motor 电机对象
 * @param target_current 目标电流
 */
void MotorDji_SetTargetCurrent(MotorDji_t *motor, float target_current)
{
    if (motor == NULL) return;
    motor->target_current = target_current;
    motor->cmd_flag++;
}

/**
 * @brief 设定目标的角度, rad
 */
void MotorDji_SetTargetAngle(MotorDji_t *motor, float target_angle)
{
    if (motor == NULL) return;
    motor->target_angle = target_angle;
    motor->cmd_flag++;
}

/**
 * @brief 设定目标的速度, rad/s
 */
void MotorDji_SetTargetOmega(MotorDji_t *motor, float target_omega)
{
    if (motor == NULL) return;
    motor->target_omega = target_omega;
    motor->cmd_flag++;
}

/**
 * @brief 设定前馈的速度, rad/s
 */
void MotorDji_SetFeedforwardOmega(MotorDji_t *motor, float feedforward_omega)
{
    if (motor == NULL) return;
    motor->feedforward_omega = feedforward_omega;
    motor->cmd_flag++;
}

/**
 * @brief 设定电机控制方式
 */
void MotorDji_SetControlMethod(MotorDji_t *motor, MotorDjiControlMethod_e control_method)
{
    if (motor == NULL) return;
    motor->control_method = control_method;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void MotorDji_CanRxCallback(MotorDji_t *motor, const uint8_t *rx_data)
{
    if (motor == NULL || motor->tx_data == NULL || rx_data == NULL) return;
    // 滑动窗口, 判断电机是否在线
    motor->rx_flag++;
    MotorDji_ProcessData(motor, rx_data);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 */
void MotorDji_CheckAlive100ms(MotorDji_t *motor)
{
    if (motor == NULL) return;

    // 判断该时间段内是否接收过电机数据
    if (motor->rx_flag == motor->last_rx_flag)
    {
        // 电机断开连接
        motor->status = MOTOR_DJI_STATUS_DISABLE;
        Pid_SetIntegralError(&motor->angle_pid, 0.0f);
        Pid_SetIntegralError(&motor->omega_pid, 0.0f);
    }
    else
    {
        // 电机保持连接
        motor->status = MOTOR_DJI_STATUS_ENABLE;
    }
    motor->last_rx_flag = motor->rx_flag;

    if (motor->cmd_flag == motor->last_cmd_flag)
    {
        // 电机未接收到新的控制指令
        Pid_SetIntegralError(&motor->angle_pid, 0.0f);
        Pid_SetIntegralError(&motor->omega_pid, 0.0f);
        motor->cmd_online = 0;
    }
    else
    {
        motor->cmd_online = 1;
    }
    motor->last_cmd_flag = motor->cmd_flag;
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 */
void MotorDji_Calculate(MotorDji_t *motor)
{
    if (motor == NULL || motor->tx_data == NULL) return;

    if (motor->status == MOTOR_DJI_STATUS_ENABLE && motor->cmd_online)
    {
        MotorDji_CalculateControl(motor);
        motor->output_value = motor->reverse ? -motor->target_current : motor->target_current;
    }
    else
    {
        motor->output_value = 0.0f;
    }
    MotorDji_Output(motor);
}

/**
 * @brief 数据处理过程
 */
static void MotorDji_ProcessData(MotorDji_t *motor, const uint8_t *rx_data)
{
    int16_t delta_encoder;

    // 计算电机本身信息
    motor->rx_data.encoder = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
    motor->rx_data.omega = (int16_t)(rx_data[2] << 8 | rx_data[3]) *
                           RPM_TO_RADPS / motor->gearbox_rate;
    motor->rx_data.current = (int16_t)(rx_data[4] << 8 | rx_data[5]);
    motor->rx_data.temperature = rx_data[6];

    // 反向安装处理
    if (motor->reverse)
    {
        motor->rx_data.encoder = (8192 - motor->rx_data.encoder) % 8192;
        motor->rx_data.omega = -motor->rx_data.omega;
        motor->rx_data.current = -motor->rx_data.current;
    }

    // 初始化预备编码器值
    if (motor->rx_flag == 1)
    {
        motor->rx_data.last_encoder = motor->rx_data.encoder;
    }

    // 计算圈数与总编码器值
    delta_encoder = motor->rx_data.encoder - motor->rx_data.last_encoder;
    if (delta_encoder < -8192 / 2)
    {
        // 正方向转过了一圈
        motor->rx_data.round_count++;
    }
    else if (delta_encoder > 8192 / 2)
    {
        // 反方向转过了一圈
        motor->rx_data.round_count--;
    }

    // 计算角度
    motor->rx_data.total_angle =
        (motor->rx_data.round_count + (float)motor->rx_data.encoder / 8192.0f) *
        2.0f * PI / motor->gearbox_rate;
    motor->rx_data.angle = wrap_center_float(motor->rx_data.total_angle, 2.0f * PI);

    // 存储预备信息
    motor->rx_data.last_encoder = motor->rx_data.encoder;
}

/**
 * @brief 计算PID
 */
static void MotorDji_CalculateControl(MotorDji_t *motor)
{
    switch (motor->control_method)
    {
    case MOTOR_DJI_CONTROL_METHOD_CURRENT:
        break;

    case MOTOR_DJI_CONTROL_METHOD_OMEGA:
        Pid_SetTarget(&motor->omega_pid, motor->target_omega + motor->feedforward_omega);
        Pid_SetFeedback(&motor->omega_pid, motor->rx_data.omega);
        Pid_Calculate(&motor->omega_pid);
        motor->target_current = Pid_GetOutput(&motor->omega_pid);
        break;

    case MOTOR_DJI_CONTROL_METHOD_ANGLE:
        motor->calculate_mod5++;
        if (motor->calculate_mod5 >= 5)
        {
            motor->calculate_mod5 = 0;
            Pid_SetTarget(&motor->angle_pid, motor->target_angle);
            Pid_SetFeedback(&motor->angle_pid, motor->rx_data.total_angle);
            Pid_Calculate(&motor->angle_pid);
        }

        motor->target_omega = Pid_GetOutput(&motor->angle_pid);
        Pid_SetTarget(&motor->omega_pid, motor->target_omega + motor->feedforward_omega);
        Pid_SetFeedback(&motor->omega_pid, motor->rx_data.omega);
        Pid_Calculate(&motor->omega_pid);
        motor->target_current = Pid_GetOutput(&motor->omega_pid);
        break;

    default:
        break;
    }
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 */
static void MotorDji_Output(MotorDji_t *motor)
{
    // 填充CAN发送数据；电流原始值限幅后转为有符号16位大端格式
    int16_t current = (int16_t)clamp_float(motor->output_value, -16384.0f, 16384.0f);
    uint16_t raw = (uint16_t)current;
    motor->tx_data[0] = (uint8_t)(raw >> 8);
    motor->tx_data[1] = (uint8_t)raw;
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
