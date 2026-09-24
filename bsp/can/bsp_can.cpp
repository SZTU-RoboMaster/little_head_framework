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

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

CanManageObject can1_manage_obj = {0};
CanManageObject can2_manage_obj = {0};
CanManageObject can3_manage_obj = {0};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 配置CAN的过滤器
 *
 * @param hfdcan CAN编号
 * @param object_param 编号 | FIFOx | ID类型 | 帧类型
 * @param id id
 * @param mask_id 屏蔽位(0x3ff, 0x1fffffff)
 */
void can_filter_mask_config(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef can_filter_init_structure;

    // 配置fifo0全通滤波器
    can_filter_init_structure.IdType = FDCAN_STANDARD_ID;
    can_filter_init_structure.FilterIndex = 0;
    can_filter_init_structure.FilterType = FDCAN_FILTER_MASK;
    can_filter_init_structure.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_filter_init_structure.FilterID1 = 0x00000000;
    can_filter_init_structure.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(hfdcan, &can_filter_init_structure);

    // 全局滤波器, 直接拒绝不符合规则的标准数据帧, 扩展数据帧, 标准遥控帧, 扩展遥控帧
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);

    // 启动CAN中断与总线
    HAL_FDCAN_ActivateNotification(hfdcan,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF |
                                       FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ARB_PROTOCOL_ERROR |
                                       FDCAN_IT_DATA_PROTOCOL_ERROR,
                                   0);
}

/**
 * @brief 初始化CAN总线
 *
 * @param hfdcan CAN编号
 * @param callback_func 处理回调函数
 */
void can_init(FDCAN_HandleTypeDef *hfdcan, can_callback_t callback_func)
{
    if (hfdcan->Instance == FDCAN1)
    {
        can1_manage_obj.can_handle = hfdcan;
        can1_manage_obj.callback_func = callback_func;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        can2_manage_obj.can_handle = hfdcan;
        can2_manage_obj.callback_func = callback_func;
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        can3_manage_obj.can_handle = hfdcan;
        can3_manage_obj.callback_func = callback_func;
    }
    can_filter_mask_config(hfdcan);

    HAL_FDCAN_Start(hfdcan);
}

/**
 * @brief 发送CAN数据
 *
 * @param hfdcan CAN编号
 * @param id id
 * @param data 被发送的数据指针
 * @param length 数据长度
 * @return uint8_t 发送状态
 */
uint8_t can_data_send(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint16_t length)
{
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = length;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data));
}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hfdcan CAN编号
 * @param RxFifo0ITs FIFO0中断状态
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // 判断程序初始化完成
    if (!initialized)
    {
        // 也得接收, 防止FIFO满
        if (hfdcan->Instance == FDCAN1)
        {
            while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can1_manage_obj.rx_buffer.header,
                                          can1_manage_obj.rx_buffer.data) == HAL_OK)
            {
            }
        }
        else if (hfdcan->Instance == FDCAN2)
        {
            while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can2_manage_obj.rx_buffer.header,
                                          can2_manage_obj.rx_buffer.data) == HAL_OK)
            {
            }
        }
        else if (hfdcan->Instance == FDCAN3)
        {
            while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can3_manage_obj.rx_buffer.header,
                                          can3_manage_obj.rx_buffer.data) == HAL_OK)
            {
            }
        }
        return;
    }

    // 选择回调函数
    if (hfdcan->Instance == FDCAN1)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can1_manage_obj.rx_buffer.header,
                                      can1_manage_obj.rx_buffer.data) == HAL_OK)
        {

            if (can1_manage_obj.callback_func != nullptr)
            {
                can1_manage_obj.callback_func(&can1_manage_obj.rx_buffer);
            }
        }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can2_manage_obj.rx_buffer.header,
                                      can2_manage_obj.rx_buffer.data) == HAL_OK)
        {
            if (can2_manage_obj.callback_func != nullptr)
            {
                can2_manage_obj.callback_func(&can2_manage_obj.rx_buffer);
            }
        }
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can3_manage_obj.rx_buffer.header,
                                      can3_manage_obj.rx_buffer.data) == HAL_OK)
        {
            if (can3_manage_obj.callback_func != nullptr)
            {
                can3_manage_obj.callback_func(&can3_manage_obj.rx_buffer);
            }
        }
    }
}

/**
 * @brief HAL库CAN错误中断
 * @note  进入BUS-off后硬件会自动让CCCR.INIT置1, 需要手动清除
 * @note  ref:
 * https://community.st.com/stm32-mcus-products-25/stm32g431-fdcan-get-into-bus-off-status-126363
 *
 * @brief HAL库CAN错误中断回调函数
 * @param hfdcan CAN编号
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    // 判断是否进入BUS-off状态
    if ((hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0U)
    {
        // 手动清除CCCR.INIT位
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
