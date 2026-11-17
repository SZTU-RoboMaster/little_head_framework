/**
 * @file bsp_can.c
 * @author anchengc
 * @brief 移植USTC-RoboWalker的CAN通信初始化与配置流程
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#include "bsp_can.h"

BspCan_Manage_t bsp_can1 = {0};
BspCan_Manage_t bsp_can2 = {0};
BspCan_Manage_t bsp_can3 = {0};

static void can_filter_mask_config(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x00000000;
    filter.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(hfdcan,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF |
                                       FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ARB_PROTOCOL_ERROR |
                                       FDCAN_IT_DATA_PROTOCOL_ERROR,
                                   0);
}

void BspCan_Init(FDCAN_HandleTypeDef *hfdcan, BspCan_Callback_t callback_func)
{
    if (hfdcan->Instance == FDCAN1)
    {
        bsp_can1.can_handle = hfdcan;
        bsp_can1.callback_func = callback_func;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        bsp_can2.can_handle = hfdcan;
        bsp_can2.callback_func = callback_func;
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        bsp_can3.can_handle = hfdcan;
        bsp_can3.callback_func = callback_func;
    }

    can_filter_mask_config(hfdcan);
    HAL_FDCAN_Start(hfdcan);
}

uint8_t BspCan_Send(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint16_t length)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = length;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data);
}

static BspCan_Manage_t *get_can_manage(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN1)
    {
        return &bsp_can1;
    }
    if (hfdcan->Instance == FDCAN2)
    {
        return &bsp_can2;
    }
    if (hfdcan->Instance == FDCAN3)
    {
        return &bsp_can3;
    }
    return NULL;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    (void)rx_fifo0_its;
    BspCan_Manage_t *manage = get_can_manage(hfdcan);
    if (manage == NULL)
    {
        return;
    }

    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &manage->rx_buffer.header,
                                  manage->rx_buffer.data) == HAL_OK)
    {
        if (initialized && manage->callback_func != NULL)
        {
            manage->callback_func(&manage->rx_buffer);
        }
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if ((hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0U)
    {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
