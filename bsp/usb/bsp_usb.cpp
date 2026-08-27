/**
 * @file bsp_usb.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_usb.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

UsbManageObject usb_manage_obj = {0};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化USB
 *
 */
void usb_init(usb_callback_t callback)
{
    usb_manage_obj.callback_func = callback;
}

/**
 * @brief USB接收回调函数
 *
 * @param buf 接收的数据
 * @param len 数据长度
 */
void USB_Rx_Callback(uint8_t *buf, uint32_t len)
{
    if (!initialized)
    {
        return;
    }
    if (usb_manage_obj.callback_func != nullptr)
    {
        usb_manage_obj.callback_func(buf, len);
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
