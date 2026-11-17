/**
 * @file referee.c
 * @author anchengc
 * @brief 裁判系统实现
 * @version 0.1
 * @date 2026-06-11 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "referee.h"

#include "CRC8_CRC16.h"

#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Referee_t referee;

// static UnpackData referee_unpack_obj;

/* Private function declarations ---------------------------------------------*/
static void Referee_ProcessData(Referee_t *self, uint8_t *rx_data, uint16_t length);
static void Referee_HandleData(Referee_t *self, uint8_t *frame);

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 裁判系统初始化
 *
 * @param __huart 指定的UART
 * @param __Frame_Header 数据包头标
 */
void Referee_Init(Referee_t *self, UART_HandleTypeDef *huart)
{
    self->referee_trust_status = REFEREE_DATA_STATUS_ENABLE;
    if (huart->Instance == USART1)
    {
        self->uart_manage_obj = &bsp_uart1;
    }
    else if (huart->Instance == USART3)
    {
        self->uart_manage_obj = &bsp_uart3;
    }
    else if (huart->Instance == UART5)
    {
        self->uart_manage_obj = &bsp_uart5;
    }
    Create_Publisher(&self->publisher, kRefereeTopicName, sizeof(self->referee_msg));
}

/**
 * @brief UART通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Referee_UartRxCallback(Referee_t *self, uint8_t *rx_data, uint16_t length)
{
    // 滑动窗口, 判断裁判系统是否在线
    self->rx_flag += 1;

    Referee_ProcessData(self, rx_data, length);

    self->referee_msg = (Referee_Message_t){
        .shooter_17mm_barrel_heat = self->power_heat_data.shooter_17mm_barrel_heat,
        .shooter_barrel_heat_limit = self->robot_state.shooter_barrel_heat_limit,
        .shooter_barrel_cooling_value = self->robot_state.shooter_barrel_cooling_value,
        .chassis_power_limit = self->robot_state.chassis_power_limit,
        .buffer_energy = self->power_heat_data.buffer_energy,
    };
    Message_PublishFromISR(&self->publisher, &self->referee_msg, sizeof(self->referee_msg));
}

/**
 * @brief TIM定时器中断定期检测裁判系统是否存活
 *
 */
void Referee_CheckAlive1000ms(Referee_t *self)
{
    // 判断该时间段内是否接收过裁判系统数据
    if (self->rx_flag == self->last_rx_flag)
    {
        // 裁判系统断开连接
        self->status = REFEREE_STATUS_DISABLE;

        BspUart_Reinit(self->uart_manage_obj->uart_handle);
    }
    else
    {
        // 裁判系统保持连接
        self->status = REFEREE_STATUS_ENABLE;
    }
    self->last_rx_flag = self->rx_flag;
}

/**
 * @brief 解包状态机
 *
 */
static void Referee_ProcessData(Referee_t *self, uint8_t *rx_data, uint16_t length)
{
    uint8_t byte = 0;
    uint8_t sof = REF_PROTOCOL_HEADER;
    UnpackData *p_obj = &self->referee_unpack_obj;

    for (int i = 0; i < length; i++)
    {
        byte = rx_data[i];

        switch (p_obj->unpack_step)
        {
        case STEP_HEADER_SOF:
        {
            if (byte == sof)
            {
                p_obj->unpack_step = STEP_LENGTH_LOW;
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            else
            {
                p_obj->index = 0;
            }
        }
        break;

        case STEP_LENGTH_LOW:
        {
            p_obj->data_len = byte;
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH:
        {
            p_obj->data_len |= (byte << 8);
            p_obj->protocol_packet[p_obj->index++] = byte;
            if (p_obj->data_len < REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)
            {
                p_obj->unpack_step = STEP_FRAME_SEQ;
            }
            else
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
        }
        break;

        case STEP_FRAME_SEQ:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_HEADER_CRC8;
        }
        break;

        case STEP_HEADER_CRC8:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;
            if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
            {
                if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
                {
                    p_obj->unpack_step = STEP_DATA_CRC16;
                }
                else
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
        }
        break;

        case STEP_DATA_CRC16:
        {
            if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;

                if (verify_CRC16_check_sum(p_obj->protocol_packet,
                                           REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    Referee_HandleData(self, p_obj->protocol_packet);
                }
            }
        }
        break;

        default:
        {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
        }
        break;
        }
    }
}

/**
 * @brief 数据处理函数
 * @param frame 接收到的数据帧
 */
static void Referee_HandleData(Referee_t *self, uint8_t *frame)
{
    // 数据处理过程
    uint16_t cmd_id = (frame[6] << 8) | frame[5];

    switch (cmd_id)
    {
        // clang-format off
    case (REFEREE_CMD_ID_GAME_STATUS):
    {
        memcpy(&self->game_status, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataGameStatus));
        break;
    }
    case (REFEREE_CMD_ID_GAME_RESULT):
    {
        memcpy(&self->game_result, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataGameResult));
        break;
    }
    case (REFEREE_CMD_ID_GAME_ROBOT_HP):
    {
        memcpy(&self->game_robot_hp, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataGameRobotHp));
        break;
    }
    case (REFEREE_CMD_ID_EVENT_DATA):
    {
        memcpy(&self->event_data, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataEventData));
        break;
    }
    case (REFEREE_CMD_ID_REFEREE_WARNING):
    {
        memcpy(&self->referee_warning, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataRefereeWarning));
        break;
    }
    case (REFEREE_CMD_ID_DART_LAUNCHING_STATUS):
    {
        memcpy(&self->dart_info, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataDartInfo));
        break;
    }
    case (REFEREE_CMD_ID_GAME_ROBOT_STATE):
    {
        memcpy(&self->robot_state, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataRobotState));
        break;
    }
    case (REFEREE_CMD_ID_POWER_HEAT_DATA):
    {
        memcpy(&self->power_heat_data, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataPowerHeatData));
        break;
    }
    case (REFEREE_CMD_ID_GAME_ROBOT_POS):
    {
        memcpy(&self->robot_pos, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataRobotPos));
        break;
    }
    case (REFEREE_CMD_ID_BUFF):
    {
        memcpy(&self->buff, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataBuff));
        break;
    }
    case (REFEREE_CMD_ID_ROBOT_HURT):
    {
        memcpy(&self->hurt_data, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataHurtData));
        break;
    }
    case (REFEREE_CMD_ID_SHOOT_DATA):
    {
        memcpy(&self->shoot_data, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataShootData));
        break;
    }
    case (REFEREE_CMD_ID_PROJECTILE_ALLOWANCE):
    {
        memcpy(&self->projectile_allowance, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataProjectileAllowance));
        break;
    }
    case (REFEREE_CMD_ID_ROBOT_RFID):
    {
        memcpy(&self->rfid_status, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataRfidStatus));
        break;
    }
    case (REFEREE_CMD_ID_ROBOT_DART_COMMAND):
    {
        memcpy(&self->dart_client_cmd, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataDartClientCmd));
        break;
    }
    case (REFEREE_CMD_ID_GROUND_ROBOT_LOCATION):
    {
        memcpy(&self->ground_robot_position, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataGroundRobotPosition));
        break;
    }
    case (REFEREE_CMD_ID_RADAR_TRACKING_PROGRESS):
    {
        memcpy(&self->radar_mark_data, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataRadarMarkData));
        break;
    }
    case (REFEREE_CMD_ID_SENTRY_DECISION_SYNC):
    {
        memcpy(&self->sentry_info, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataSentryInfo));
        break;
    }
    case (REFEREE_CMD_ID_RADAR_DECISION_SYNC):
    {
        memcpy(&self->radar_info, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataRadarInfo));
        break;
    }
    case (REFEREE_CMD_ID_ROBOT_RECEIVE_CUSTOM_CONTROLLER_DATA):
    {
        memcpy(&self->custom_robot_data, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataCustomRobotData));
        break;
    }
    case (REFEREE_CMD_ID_CLIENT_MINIMAP_INTERACTIVE_DATA):
    {
        memcpy(&self->map_command_data, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataMapCommandData));
        break;
    }
    case (REFEREE_CMD_ID_ROBOT_RECEIVE_CUSTOM_CLIENT_DATA):
    {
        memcpy(&self->client_robot_data, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataClientRobotData));
        break;
    }
    default:
        break;
        // clang-format on
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
