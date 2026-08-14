/**
 * @file vt13.h
 * @author anchengc
 * @brief 遥控器VT13头文件
 * @version 0.1
 * @date 2026-06-10 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"

#include "message_center.h"
#include "message_def.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 遥控器VT13状态
 *
 */
enum Vt13Status
{
    VT13_STATUS_DISABLE = 0,
    VT13_STATUS_ENABLE,
};



/**
 * @brief Specialized, 遥控器VT13
 *
 */
class Vt13
{
public:
    void init(UART_HandleTypeDef *huart);

    void uart_rx_callback(uint8_t *rx_data, uint16_t length);

    void check_alive_100ms();

protected:
    // 初始化相关常量

    // 绑定的UART
    UartManageObject *uart_manage_obj_;

    // 常量

    // 内部变量

    // 当前时刻的遥控器VT13接收flag
    uint32_t rx_flag_ = 0;
    // 前一时刻的遥控器VT13接收flag
    uint32_t last_rx_flag_ = 0;

    // 读变量

    // 遥控器VT13状态
    Vt13Status vt13_status_ = VT13_STATUS_DISABLE;
    // 遥控器VT13对外接口信息
    Vt13Message data_;

    // 写变量
    Publisher<Vt13Message> publisher_;

    // 读写变量

    // 内部函数

    void process_data(uint8_t *rx_data, uint16_t length);
};
/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
