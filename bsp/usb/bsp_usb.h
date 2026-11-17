/**
 * @file bsp_usb.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-07-10 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief USB通信接收回调函数数据类型
 *
 */
typedef void (*BspUsb_Callback_t)(uint8_t *buf, uint32_t len);

/**
 * @brief USB通信处理结构体
 */
typedef struct
{
    BspUsb_Callback_t callback_func;
} BspUsb_Manage_t;

/* Exported variables ---------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t initialized;

extern BspUsb_Manage_t bsp_usb;

/* Exported function declarations ---------------------------------------------*/

void BspUsb_Init(BspUsb_Callback_t callback);

void BspUsb_RxCallback(uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
