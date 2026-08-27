/**
 * @file referee.h
 * @author anchengc
 * @brief 裁判系统头文件
 * @version 0.1
 * @date 2026-06-11 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"
#include "referee_protocol.h"

#include "message_center.h"
#include "message_def.h"

/* Exported macros -----------------------------------------------------------*/

static constexpr uint8_t REF_PROTOCOL_HEADER = 0xA5;
static constexpr uint16_t REF_PROTOCOL_HEADER_SIZE = sizeof(FrameHeader);
static constexpr uint16_t REF_PROTOCOL_CMD_SIZE = 2;
static constexpr uint16_t REF_PROTOCOL_CRC16_SIZE = 2;
static constexpr uint16_t REF_HEADER_CRC_LEN = REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE;
static constexpr uint16_t REF_HEADER_CRC_CMDID_LEN =
    REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t);
static constexpr uint16_t REF_HEADER_CMDID_LEN = REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t);
static constexpr uint16_t REF_PROTOCOL_FRAME_MAX_SIZE = 255;
static constexpr uint16_t REF_PROTOCOL_CMD_MAX_NUM = 20;

/* Exported types ------------------------------------------------------------*/

struct __attribute__((packed)) RefereeUartData
{
    uint8_t frame_header;
    uint16_t data_length;
    uint8_t sequence;
    uint8_t crc8;
    uint16_t cmd_id;
    uint8_t data[1];
};

/**
 * @brief 解包步骤枚举值
 *
 */
enum UnpackStep
{
    STEP_HEADER_SOF = 0,
    STEP_LENGTH_LOW = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16 = 5,
};

struct UnpackData
{
    UnpackStep unpack_step;
    uint16_t data_len;
    uint16_t index;
    uint8_t protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
};

/**
 * @brief 裁判系统状态
 *
 */
enum RefereeStatus
{
    REFEREE_STATUS_DISABLE = 0,
    REFEREE_STATUS_ENABLE,
};

/**
 * @brief 各种标签, 场地, 相关设施激活与存活状态
 *
 */
enum RefereeDataStatus
{
    REFEREE_DATA_STATUS_DISABLE = 0,
    REFEREE_DATA_STATUS_ENABLE,
};

/**
 * @brief Specialized, 裁判系统
 *
 */
class Referee
{
public:
    void init(UART_HandleTypeDef *huart);

    void uart_rx_callback(uint8_t *rx_data, uint16_t length);

    void check_alive_1000ms();

    // 裁判系统状态
    RefereeStatus status_ = REFEREE_STATUS_DISABLE;

protected:
    // 初始化相关常量

    // 绑定的UART
    UartManageObject *uart_manage_obj_;

    // 常量

    // 内部变量

    // 当前时刻的裁判系统接收flag
    uint32_t rx_flag_ = 0;
    // 前一时刻的裁判系统接收flag
    uint32_t last_rx_flag_ = 0;

    // 发送序列号
    uint8_t sequence_ = 0;

    // UI是否是初次绘制, 没绘制过是0
    uint8_t ui_change_flag_[10][10] = {0};

    // 比赛状态
    RefereeRxDataGameStatus game_status_;
    // 比赛结果
    RefereeRxDataGameResult game_result_;
    // 机器人血量
    RefereeRxDataGameRobotHp game_robot_hp_;

    // 场地事件
    RefereeRxDataEventData event_data_;
    // 裁判警告信息
    RefereeRxDataRefereeWarning referee_warning_;
    // 飞镖发射相关数据
    RefereeRxDataDartInfo dart_info_;

    // 机器人状态
    RefereeRxDataRobotState robot_state_;
    // 当前机器人实时功率热量
    RefereeRxDataPowerHeatData power_heat_data_;
    // 当前机器人实时位置
    RefereeRxDataRobotPos robot_pos_;
    // 当前机器人增益
    RefereeRxDataBuff buff_;
    // 伤害情况
    RefereeRxDataHurtData hurt_data_;
    // 子弹信息
    RefereeRxDataShootData shoot_data_;
    // 子弹剩余信息
    RefereeRxDataProjectileAllowance projectile_allowance_;
    // RFID状态信息
    RefereeRxDataRfidStatus rfid_status_;
    // 飞镖选手端指令
    RefereeRxDataDartClientCmd dart_client_cmd_;
    // 哨兵获取己方位置信息
    RefereeRxDataGroundRobotPosition ground_robot_position_;
    // 雷达标记进度
    RefereeRxDataRadarMarkData radar_mark_data_;
    // 哨兵决策信息
    RefereeRxDataSentryInfo sentry_info_;
    // 雷达决策信息
    RefereeRxDataRadarInfo radar_info_;

    // 自定义控制器与机器人交互数据
    RefereeRxDataCustomRobotData custom_robot_data_;
    // 选手端小地图交互数据
    RefereeRxDataMapCommandData map_command_data_;
    // 自定义客户端发送给机器人的自定义指令
    RefereeRxDataClientRobotData client_robot_data_;

    // 读变量

    // 写变量
    RefereeTxDataInteractionFigure graphic_config_[10][10];

    Publisher<RefereeMessage> publisher_;
    RefereeMessage referee_msg_;

    // 读写变量

    // 解包状态机数据
    UnpackData referee_unpack_obj_;

    // 裁判系统是否可信
    RefereeDataStatus referee_trust_status_ = REFEREE_DATA_STATUS_ENABLE;

    // 内部函数

    void process_data(uint8_t *rx_data, uint16_t length);

    void handle_data(uint8_t *frame);
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
