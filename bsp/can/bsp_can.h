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

#include "can.h"
#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN接收的信息结构体
 *
 */
struct CanRxBuffer
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
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
    CAN_HandleTypeDef *can_handle;
    CanRxBuffer rx_buffer;
    can_callback_t callback_func;
};

/* Exported variables ---------------------------------------------------------*/

extern uint8_t initialized;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern CanManageObject can1_manage_obj;
extern CanManageObject can2_manage_obj;

/* Exported function declarations ---------------------------------------------*/
void can_init(CAN_HandleTypeDef *hcan, can_callback_t callback_func);

uint8_t can_data_send(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint16_t length);

/************************ COPYRIGHT(C) SZTU-HJ **************************/
