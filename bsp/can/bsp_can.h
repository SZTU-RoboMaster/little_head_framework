/**
 * @file bsp_can.h
 * @author anchengc
 * @brief 移植USTC-RoboWalker的CAN通信初始化与配置流程
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN接收的信息结构体
 *
 */
struct CanRxBuffer
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
};

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*can_callback_t)(CanRxBuffer *);

/**
 * @brief CAN通信处理结构体
 *
 */
struct CanManageObject
{
    FDCAN_HandleTypeDef *can_handle;
    CanRxBuffer rx_buffer;
    can_callback_t callback_func;
};

/* Exported variables ---------------------------------------------------------*/

extern uint8_t initialized;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern CanManageObject can1_manage_obj;
extern CanManageObject can2_manage_obj;
extern CanManageObject can3_manage_obj;

/* Exported function declarations ---------------------------------------------*/
void can_init(FDCAN_HandleTypeDef *hfdcan, can_callback_t callback_func);

uint8_t can_data_send(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint16_t length);

/************************ COPYRIGHT(C) SZTU-HJ **************************/
