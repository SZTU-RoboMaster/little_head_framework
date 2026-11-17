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

#include "fdcan.h"

typedef struct
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
} BspCan_RxBuffer_t;

typedef void (*BspCan_Callback_t)(BspCan_RxBuffer_t *);

typedef struct
{
    FDCAN_HandleTypeDef *can_handle;
    BspCan_RxBuffer_t rx_buffer;
    BspCan_Callback_t callback_func;
} BspCan_Manage_t;

extern uint8_t initialized;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern BspCan_Manage_t bsp_can1;
extern BspCan_Manage_t bsp_can2;
extern BspCan_Manage_t bsp_can3;

void BspCan_Init(FDCAN_HandleTypeDef *hfdcan, BspCan_Callback_t callback_func);
uint8_t BspCan_Send(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint16_t length);

/************************ COPYRIGHT(C) SZTU-HJ **************************/
