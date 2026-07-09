/**
 * @file bsp_can.cpp
 * @author anchengc
 * @brief 移植USTC-RoboWalker的CAN通信初始化与配置流程
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"

/* Private macros ------------------------------------------------------------*/

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

// 标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

CanManageObject can1_manage_obj = {0};
CanManageObject can2_manage_obj = {0};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 配置CAN的过滤器
 *
 * @param hcan CAN编号
 * @param object_param 编号 | FIFOx | ID类型 | 帧类型
 * @param id id
 * @param mask_id 屏蔽位(0x3ff, 0x1fffffff)
 */
void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t object_param, uint32_t id,
                            uint32_t mask_id)
{
    CAN_FilterTypeDef can_filter_init_structure;

    // 检测传参是否正确
    assert_param(hcan != NULL);

    if ((object_param & 0x02))
    {
        // 标准帧
        // 掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = id << 3 >> 16;
        // 掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = id << 3 | ((object_param & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = mask_id << 3 << 16;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = mask_id << 3 | ((object_param & 0x03) << 1);
    }
    else
    {
        // 扩展帧
        // 掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = id << 5;
        // 掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ((object_param & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = mask_id << 5;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = ((object_param & 0x03) << 1);
    }
    // 滤波器序号, 0-27, 共28个滤波器, 前14个在CAN1, 后14个在CAN2
    can_filter_init_structure.FilterBank = object_param >> 3;
    // 滤波器绑定FIFO0
    can_filter_init_structure.FilterFIFOAssignment = (object_param >> 2) & 0x01;
    // 使能滤波器
    can_filter_init_structure.FilterActivation = ENABLE;
    // 滤波器模式，设置ID掩码模式
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 32位滤波
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    // 从机模式选择开始单元
    can_filter_init_structure.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param callback_func 处理回调函数
 */
void can_init(CAN_HandleTypeDef *hcan, can_callback_t callback_func)
{
    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    if (hcan->Instance == CAN1)
    {
        can1_manage_obj.can_handle = hcan;
        can1_manage_obj.callback_func = callback_func;

        can_filter_mask_config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        can_filter_mask_config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    else if (hcan->Instance == CAN2)
    {
        can2_manage_obj.can_handle = hcan;
        can2_manage_obj.callback_func = callback_func;

        can_filter_mask_config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        can_filter_mask_config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
}

/**
 * @brief 发送CAN数据
 *
 * @param hcan CAN编号
 * @param id id
 * @param data 被发送的数据指针
 * @param length 数据长度
 * @return uint8_t 发送状态
 */
uint8_t can_data_send(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint16_t length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    // 检测传参是否正确
    assert_param(hcan != NULL);

    tx_header.StdId = id;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = length;

    return (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &used_mailbox));
}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 判断程序初始化完成
    if (!init_finished)
    {
        return;
    }

    // 选择回调函数
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &can1_manage_obj.rx_buffer.header,
                             can1_manage_obj.rx_buffer.data);
        if (can1_manage_obj.callback_func != nullptr)
        {
            can1_manage_obj.callback_func(&can1_manage_obj.rx_buffer);
        }
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &can2_manage_obj.rx_buffer.header,
                             can2_manage_obj.rx_buffer.data);
        if (can2_manage_obj.callback_func != nullptr)
        {
            can2_manage_obj.callback_func(&can2_manage_obj.rx_buffer);
        }
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 判断程序初始化完成
    if (!init_finished)
    {
        return;
    }

    // 选择回调函数
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &can1_manage_obj.rx_buffer.header,
                             can1_manage_obj.rx_buffer.data);
        if (can1_manage_obj.callback_func != nullptr)
        {
            can1_manage_obj.callback_func(&can1_manage_obj.rx_buffer);
        }
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &can2_manage_obj.rx_buffer.header,
                             can2_manage_obj.rx_buffer.data);
        if (can2_manage_obj.callback_func != nullptr)
        {
            can2_manage_obj.callback_func(&can2_manage_obj.rx_buffer);
        }
    }
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
