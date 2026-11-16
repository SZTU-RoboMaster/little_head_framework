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

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN接收的信息结构体
 *
 */
typedef struct
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} BspCan_RxBuffer_t;

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*BspCan_Callback_t)(BspCan_RxBuffer_t *);

/**
 * @brief CAN通信处理结构体
 *
 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;
    BspCan_RxBuffer_t rx_buffer;
    BspCan_Callback_t callback_func;
} BspCan_Manage_t;

/* Exported variables ---------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t initialized;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern BspCan_Manage_t bsp_can1;
extern BspCan_Manage_t bsp_can2;

/* Exported function declarations ---------------------------------------------*/
void BspCan_Init(CAN_HandleTypeDef *hcan, BspCan_Callback_t callback_func);

uint8_t BspCan_Send(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

/************************ COPYRIGHT(C) SZTU-HJ **************************/
