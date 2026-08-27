/**
 * @file dr16.h
 * @author anchengc
 * @brief 遥控器DR16头文件
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
 * @brief 遥控器DR16状态
 *
 */
enum Dr16Status
{
    DR16_STATUS_DISABLE = 0,
    DR16_STATUS_ENABLE,
};

/**
 * @brief Specialized, 遥控器DR16
 *
 */
class Dr16
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

    // 当前时刻的遥控器接收flag
    uint32_t rx_flag_ = 0;
    // 前一时刻的遥控器接收flag
    uint32_t last_rx_flag_ = 0;

    // 读变量

    // 遥控器状态
    Dr16Status dr16_status_ = DR16_STATUS_DISABLE;
    // 遥控器数据
    Dr16Message data_;

    // 写变量
    Publisher<Dr16Message> publisher_;

    // 读写变量

    // 内部函数

    void process_data(uint8_t *rx_data, uint16_t length);
};
/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
