/**
 * @file motor_dm.c
 * @author anchengc
 * @brief 达妙电机交互库
 * @version 0.1
 * @date 2026-06-03 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "motor_dm.h"

#include "math_tools.h"

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
static void MotorDm_ProcessData(MotorDm_t *self, const uint8_t *rx_data);
static void MotorDm_Output(MotorDm_t *self);
static void MotorDm_SendMitControl(MotorDm_t *self);
static void MotorDm_SendAngleOmegaControl(MotorDm_t *self);
static void MotorDm_SendOmegaControl(MotorDm_t *self);

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 电机初始化
 *
 * @param hfdcan 绑定的FDCAN总线
 * @param can_id 发数据绑定的CAN id, 是上位机驱动参数CAN_ID
 * @param master_id 收数据绑定的CAN id, 与上位机驱动参数Master_ID保持一致
 * @param control_method 电机控制方式
 * @param p_max 最大位置, 与上位机控制幅值PMAX保持一致
 * @param v_max 最大速度, 与上位机控制幅值VMAX保持一致
 * @param t_max 最大扭矩, 与上位机控制幅值TMAX保持一致
 * @param kp MIT控制的K_P
 * @param kd MIT控制的K_D
 * @param reverse 反向安装标志
 * @param enable_status 电机使能状态
 */
void MotorDm_Init(MotorDm_t *self, FDCAN_HandleTypeDef *hfdcan, uint16_t can_id,
                  uint16_t master_id,
                  MotorDmControlMethod_e control_method, float p_max, float v_max, float t_max,
                  float kp, float kd, uint8_t reverse, MotorDmEnableStatus_e enable_status)
{
    self->rx_data = (MotorDmRxData_t){0};
    self->rx_flag = 0;
    self->last_rx_flag = 0;
    self->cmd_flag = 0;
    self->last_cmd_flag = 0;
    self->cmd_online = 0;
    self->cmd_angle = 0.0f;
    self->cmd_omega = 0.0f;
    self->cmd_torque = 0.0f;
    self->target_angle = 0.0f;
    self->target_omega = 0.0f;
    self->target_torque = 0.0f;
    self->status = MOTOR_DM_STATUS_DISCONNECTED;
    if (hfdcan->Instance == FDCAN1)
    {
        self->can_manage_obj = &bsp_can1;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        self->can_manage_obj = &bsp_can2;
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        self->can_manage_obj = &bsp_can3;
    }

    self->can_rx_id = master_id;
    switch (control_method)
    {
    case (MOTOR_DM_CONTROL_METHOD_MIT):
    {
        self->can_tx_id = can_id;
        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_ANGLE_OMEGA):
    {
        self->can_tx_id = can_id + 0x100;
        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_OMEGA):
    {
        self->can_tx_id = can_id + 0x200;
        break;
    }
    }
    self->control_method = control_method;
    self->p_max = p_max;
    self->v_max = v_max;
    self->t_max = t_max;
    self->kp = kp;
    self->kd = kd;
    self->reverse = reverse;
    self->enable_status = enable_status;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param target_angle 目标的角度, rad
 */
void MotorDm_SetTargetAngle(MotorDm_t *self, float target_angle)
{
    self->target_angle = target_angle;
    self->cmd_flag++;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param target_omega 目标的速度, rad/s
 */
void MotorDm_SetTargetOmega(MotorDm_t *self, float target_omega)
{
    self->target_omega = target_omega;
    self->cmd_flag++;
}

/**
 * @brief 设定目标的扭矩, N*m
 *
 * @param target_torque 目标的扭矩, N*m
 */
void MotorDm_SetTargetTorque(MotorDm_t *self, float target_torque)
{
    self->target_torque = target_torque;
    self->cmd_flag++;
}

/**
 * @brief 设定PID参数
 *
 * @param kp kp, 0~500
 * @param kd kd, 0~5
 */
void MotorDm_SetPidParams(MotorDm_t *self, float kp, float kd)
{
    self->kp = kp;
    self->kd = kd;
}

void MotorDm_SetEnableStatus(MotorDm_t *self, MotorDmEnableStatus_e enable_status)
{
    self->enable_status = enable_status;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void MotorDm_CanRxCallback(MotorDm_t *self, const uint8_t *rx_data)
{
    // 滑动窗口, 判断电机是否在线
    self->rx_flag++;

    MotorDm_ProcessData(self, rx_data);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void MotorDm_CheckAlive100ms(MotorDm_t *self)
{
    // 判断该时间段内是否接收过电机数据
    if (self->rx_flag == self->last_rx_flag)
    {
        // 电机断开连接
        self->status = MOTOR_DM_STATUS_DISCONNECTED;
    }
    else
    {
        // 电机保持连接
        self->status = MOTOR_DM_STATUS_CONNECTED;
    }
    self->last_rx_flag = self->rx_flag;

    if (self->cmd_flag == self->last_cmd_flag)
    {
        // 电机未接收到新的控制指令, 失能电机
        self->cmd_online = 0;
    }
    else
    {
        self->cmd_online = 1;
    }
    self->last_cmd_flag = self->cmd_flag;
}

/**
 * @brief TIM定时器中断发送出去的回调函数, 计算周期取决于自主设置的控制周期
 *
 */
void MotorDm_SendControl(MotorDm_t *self)
{
    if (self->status == MOTOR_DM_STATUS_CONNECTED && self->cmd_online &&
        self->enable_status == MOTOR_DM_ENABLE_STATUS_ENABLE)
    {
        if (self->rx_data.error_status == MOTOR_DM_ERR_STATUS_ENABLE)
        {
            // 电机在线, 正常控制
            self->cmd_angle = clamp_float(self->reverse ? -self->target_angle : self->target_angle, -self->p_max, self->p_max);
            self->cmd_omega = clamp_float(self->reverse ? -self->target_omega : self->target_omega, -self->v_max, self->v_max);
            self->cmd_torque = clamp_float(self->reverse ? -self->target_torque : self->target_torque, -self->t_max, self->t_max);
            self->kp = clamp_float(self->kp, 0.0f, 500.0f);
            self->kd = clamp_float(self->kd, 0.0f, 5.0f);

            MotorDm_Output(self);
        }
        else if (self->rx_data.error_status == MOTOR_DM_ERR_STATUS_DISABLE)
        {
            // 电机失能, 发送使能帧
            MotorDm_SendEnableCmd(self);
        }
        else
        {
            // 电机异常, 发送清除错误信息帧
            MotorDm_SendClearError(self);
        }
    }
    else
    {
        MotorDm_SendDisableCmd(self);
    }
}

/**
 * @brief 数据处理过程
 *
 */
static void MotorDm_ProcessData(MotorDm_t *self, const uint8_t *rx_data)
{
    // 数据处理过程

    // 电机ID不匹配, 则不进行处理
    if ((rx_data[0] & 0x0f) != (self->can_tx_id & 0x0f))
    {
        return;
    }

    float delta_position;

    // 计算电机本身信息
    self->rx_data.error_status = (MotorDmErrStatus_e)(rx_data[0] >> 4);
    self->rx_data.position = uint_to_float(rx_data[1] << 8 | rx_data[2], -self->p_max, self->p_max, 16);
    self->rx_data.omega = uint_to_float(rx_data[3] << 4 | rx_data[4] >> 4, -self->v_max, self->v_max, 12);
    self->rx_data.torque = uint_to_float((rx_data[4] & 0x0f) << 8 | rx_data[5], -self->t_max, self->t_max, 12);
    self->rx_data.mos_temperature = rx_data[6];
    self->rx_data.rotor_temperature = rx_data[7];

    // 反向安装处理
    if (self->reverse != 0)
    {
        self->rx_data.position = -self->rx_data.position;
        self->rx_data.omega = -self->rx_data.omega;
        self->rx_data.torque = -self->rx_data.torque;
    }

    // 初始化预备位置值
    if (self->rx_flag == 1)
    {
        self->rx_data.last_position = self->rx_data.position;
    }

    // 计算圈数与总位置值
    delta_position = self->rx_data.position - self->rx_data.last_position;
    if (delta_position < -self->p_max)
    {
        // 正方向转过了一个P_MAX周期
        self->rx_data.position_cycle_count++;
    }
    else if (delta_position > self->p_max)
    {
        // 反方向转过了一个P_MAX周期
        self->rx_data.position_cycle_count--;
    }
    self->rx_data.total_angle = self->rx_data.position_cycle_count * 2.0f * self->p_max + self->rx_data.position;

    // 计算角度, 归化到 -PI ~ PI
    self->rx_data.angle = wrap_center_float(self->rx_data.total_angle, 2.0f * PI);

    // 存储预备信息
    self->rx_data.last_position = self->rx_data.position;
}

/**
 * @brief 电机数据输出到CAN总线
 *
 */
static void MotorDm_Output(MotorDm_t *self)
{
    switch (self->control_method)
    {
    case (MOTOR_DM_CONTROL_METHOD_MIT):
    {
        MotorDm_SendMitControl(self);
        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_ANGLE_OMEGA):
    {
        MotorDm_SendAngleOmegaControl(self);
        break;
    }
    case (MOTOR_DM_CONTROL_METHOD_OMEGA):
    {
        MotorDm_SendOmegaControl(self);
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
void MotorDm_SendClearError(MotorDm_t *self)
{
    BspCan_Send(self->can_manage_obj->can_handle, self->can_tx_id, dm_motor_clear_error_msg, 8);
}

/**
 * @brief 发送使能帧
 *
 */
void MotorDm_SendEnableCmd(MotorDm_t *self)
{
    BspCan_Send(self->can_manage_obj->can_handle, self->can_tx_id, dm_motor_enable_msg, 8);
}

/**
 * @brief 发送失能帧
 *
 */
void MotorDm_SendDisableCmd(MotorDm_t *self)
{
    BspCan_Send(self->can_manage_obj->can_handle, self->can_tx_id, dm_motor_disable_msg, 8);
}

/**
 * @brief 发送保存零点帧
 *
 */
void MotorDm_SaveZeroPosition(MotorDm_t *self)
{
    BspCan_Send(self->can_manage_obj->can_handle, self->can_tx_id, dm_motor_save_zero_msg, 8);
}

/**
 * @brief 发送MIT控制帧
 *
 */
static void MotorDm_SendMitControl(MotorDm_t *self)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(self->cmd_angle, -self->p_max, self->p_max, 16);
    vel_tmp = float_to_uint(self->cmd_omega, -self->v_max, self->v_max, 12);
    kp_tmp = float_to_uint(self->kp, 0.0f, 500.0f, 12);
    kd_tmp = float_to_uint(self->kd, 0.0f, 5.0f, 12);
    tor_tmp = float_to_uint(self->cmd_torque, -self->t_max, self->t_max, 12);

    self->tx_data[0] = (pos_tmp >> 8);
    self->tx_data[1] = pos_tmp;
    self->tx_data[2] = (vel_tmp >> 4);
    self->tx_data[3] = (vel_tmp & 0xf) << 4 | (kp_tmp >> 8);
    self->tx_data[4] = kp_tmp;
    self->tx_data[5] = kd_tmp >> 4;
    self->tx_data[6] = (kd_tmp & 0xf) << 4 | (tor_tmp >> 8);
    self->tx_data[7] = tor_tmp;

    BspCan_Send(self->can_manage_obj->can_handle, self->can_tx_id, self->tx_data, 8);
}

/**
 * @brief 发送位置速度控制帧
 *
 */
static void MotorDm_SendAngleOmegaControl(MotorDm_t *self)
{
    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&self->cmd_angle;
    vbuf = (uint8_t *)&self->cmd_omega;

    self->tx_data[0] = *pbuf;
    self->tx_data[1] = *(pbuf + 1);
    self->tx_data[2] = *(pbuf + 2);
    self->tx_data[3] = *(pbuf + 3);
    self->tx_data[4] = *vbuf;
    self->tx_data[5] = *(vbuf + 1);
    self->tx_data[6] = *(vbuf + 2);
    self->tx_data[7] = *(vbuf + 3);

    BspCan_Send(self->can_manage_obj->can_handle, self->can_tx_id, self->tx_data, 8);
}

/**
 * @brief 发送速度控制帧
 *
 */
static void MotorDm_SendOmegaControl(MotorDm_t *self)
{
    uint8_t *vbuf;
    vbuf = (uint8_t *)&self->cmd_omega;

    self->tx_data[0] = *vbuf;
    self->tx_data[1] = *(vbuf + 1);
    self->tx_data[2] = *(vbuf + 2);
    self->tx_data[3] = *(vbuf + 3);
    self->tx_data[4] = 0;
    self->tx_data[5] = 0;
    self->tx_data[6] = 0;
    self->tx_data[7] = 0;

    BspCan_Send(self->can_manage_obj->can_handle, self->can_tx_id, self->tx_data, 8);
}
/************************ COPYRIGHT(C) SZTU-HJ **************************/

