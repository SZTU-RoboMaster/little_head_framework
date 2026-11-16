/**
 * @file bsp_usb.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-07-10 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_usb.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

BspUsb_Manage_t bsp_usb = {0};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化USB
 *
 */
void BspUsb_Init(BspUsb_Callback_t callback)
{
    bsp_usb.callback_func = callback;
}

/**
 * @brief USB接收回调函数
 *
 * @param buf 接收的数据
 * @param len 数据长度
 */
void BspUsb_RxCallback(uint8_t *buf, uint32_t len)
{
    if (!initialized)
    {
        return;
    }
    if (bsp_usb.callback_func != NULL)
    {
        bsp_usb.callback_func(buf, len);
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
