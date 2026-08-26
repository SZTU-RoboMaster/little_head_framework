/**
 * @file referee.cpp
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
#include <cstring>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// static UnpackData referee_unpack_obj;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 裁判系统初始化
 *
 * @param __huart 指定的UART
 * @param __Frame_Header 数据包头标
 */
void Referee::init(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_manage_obj_ = &uart1_manage_obj;
    }
    else if (huart->Instance == USART3)
    {
        uart_manage_obj_ = &uart3_manage_obj;
    }
    else if (huart->Instance == USART6)
    {
        uart_manage_obj_ = &uart6_manage_obj;
    }
    publisher_ = MessageCenter::instance().advertise<RefereeMessage>(kRefereeTopicName);
}

/**
 * @brief UART通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Referee::uart_rx_callback(uint8_t *rx_data, uint16_t length)
{
    // 滑动窗口, 判断裁判系统是否在线
    rx_flag_ += 1;

    process_data(rx_data, length);

    referee_msg_ = {
        .shooter_17mm_barrel_heat = power_heat_data_.shooter_17mm_barrel_heat,
        .shooter_barrel_heat_limit = robot_state_.shooter_barrel_heat_limit,
        .shooter_barrel_cooling_value = robot_state_.shooter_barrel_cooling_value,
        .chassis_power_limit = robot_state_.chassis_power_limit,
        .buffer_energy = power_heat_data_.buffer_energy,
    };
    publisher_.publish(referee_msg_);
}

/**
 * @brief TIM定时器中断定期检测裁判系统是否存活
 *
 */
void Referee::check_alive_1000ms()
{
    // 判断该时间段内是否接收过裁判系统数据
    if (rx_flag_ == last_rx_flag_)
    {
        // 裁判系统断开连接
        status_ = REFEREE_STATUS_DISABLE;

        uart_reinit(uart_manage_obj_->uart_handle);
    }
    else
    {
        // 裁判系统保持连接
        status_ = REFEREE_STATUS_ENABLE;
    }
    last_rx_flag_ = rx_flag_;
}

/**
 * @brief 解包状态机
 *
 */
void Referee::process_data(uint8_t *rx_data, uint16_t length)
{
    uint8_t byte = 0;
    uint8_t sof = REF_PROTOCOL_HEADER;
    UnpackData *p_obj = &referee_unpack_obj_;

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
                    handle_data(p_obj->protocol_packet);
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
void Referee::handle_data(uint8_t *frame)
{
    // 数据处理过程
    uint16_t cmd_id = (frame[6] << 8) | frame[5];

    switch (cmd_id)
    {
        // clang-format off
    case (REFEREE_CMD_ID_GAME_STATUS):
    {
        std::memcpy(&game_status_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataGameStatus));
        break;
    }
    case (REFEREE_CMD_ID_GAME_RESULT):
    {
        std::memcpy(&game_result_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataGameResult));
        break;
    }
    case (REFEREE_CMD_ID_GAME_ROBOT_HP):
    {
        std::memcpy(&game_robot_hp_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataGameRobotHp));
        break;
    }
    case (REFEREE_CMD_ID_EVENT_DATA):
    {
        std::memcpy(&event_data_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataEventData));
        break;
    }
    case (REFEREE_CMD_ID_REFEREE_WARNING):
    {
        std::memcpy(&referee_warning_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataRefereeWarning));
        break;
    }
    case (REFEREE_CMD_ID_DART_LAUNCHING_STATUS):
    {
        std::memcpy(&dart_info_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataDartInfo));
        break;
    }
    case (REFEREE_CMD_ID_GAME_ROBOT_STATE):
    {
        std::memcpy(&robot_state_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataRobotState));
        break;
    }
    case (REFEREE_CMD_ID_POWER_HEAT_DATA):
    {
        std::memcpy(&power_heat_data_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataPowerHeatData));
        break;
    }
    case (REFEREE_CMD_ID_GAME_ROBOT_POS):
    {
        std::memcpy(&robot_pos_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataRobotPos));
        break;
    }
    case (REFEREE_CMD_ID_BUFF):
    {
        std::memcpy(&buff_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataBuff));
        break;
    }
    case (REFEREE_CMD_ID_ROBOT_HURT):
    {
        std::memcpy(&hurt_data_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataHurtData));
        break;
    }
    case (REFEREE_CMD_ID_SHOOT_DATA):
    {
        std::memcpy(&shoot_data_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataShootData));
        break;
    }
    case (REFEREE_CMD_ID_PROJECTILE_ALLOWANCE):
    {
        std::memcpy(&projectile_allowance_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataProjectileAllowance));
        break;
    }
    case (REFEREE_CMD_ID_ROBOT_RFID):
    {
        std::memcpy(&rfid_status_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataRfidStatus));
        break;
    }
    case (REFEREE_CMD_ID_ROBOT_DART_COMMAND):
    {
        std::memcpy(&dart_client_cmd_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataDartClientCmd));
        break;
    }
    case (REFEREE_CMD_ID_GROUND_ROBOT_LOCATION):
    {
        std::memcpy(&ground_robot_position_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataGroundRobotPosition));
        break;
    }
    case (REFEREE_CMD_ID_RADAR_TRACKING_PROGRESS):
    {
        std::memcpy(&radar_mark_data_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataRadarMarkData));
        break;
    }
    case (REFEREE_CMD_ID_SENTRY_DECISION_SYNC):
    {
        std::memcpy(&sentry_info_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataSentryInfo));
        break;
    }
    case (REFEREE_CMD_ID_RADAR_DECISION_SYNC):
    {
        std::memcpy(&radar_info_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataRadarInfo));
        break;
    }
    case (REFEREE_CMD_ID_ROBOT_RECEIVE_CUSTOM_CONTROLLER_DATA):
    {
        std::memcpy(&custom_robot_data_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataCustomRobotData));
        break;
    }
    case (REFEREE_CMD_ID_CLIENT_MINIMAP_INTERACTIVE_DATA):
    {
        std::memcpy(&map_command_data_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataMapCommandData));
        break;
    }
    case (REFEREE_CMD_ID_ROBOT_RECEIVE_CUSTOM_CLIENT_DATA):
    {
        std::memcpy(&client_robot_data_, frame + REF_HEADER_CMDID_LEN, sizeof(RefereeRxDataClientRobotData));
        break;
    }
    default:
        break;
        // clang-format on
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
