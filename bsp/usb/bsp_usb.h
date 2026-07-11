/**
 * @file bsp_usb.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-10 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief USB通信接收回调函数数据类型
 *
 */
typedef void (*usb_callback_t)(uint8_t *buf, uint32_t len);

/**
 * @brief USB通信处理结构体
 */
typedef struct
{
    usb_callback_t callback_func;
} UsbManageObject;

/* Exported variables ---------------------------------------------------------*/
extern uint8_t init_finished;

extern UsbManageObject usb_manage_obj;

/* Exported function declarations ---------------------------------------------*/

void usb_init(usb_callback_t callback);

#ifdef __cplusplus
extern "C" {
#endif

void USB_Rx_Callback(uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
