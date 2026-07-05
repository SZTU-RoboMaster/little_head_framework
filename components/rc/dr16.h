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
 * @brief  remote control information
 */
struct Dr16Data
{
    /* rocker channel information */
    int16_t ch_0;
    int16_t ch_1;
    int16_t ch_2;
    int16_t ch_3;
    /* left and right lever information */
    uint8_t sw_1;
    uint8_t sw_2;
    /* mouse movement and button information */
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t left;
        uint8_t right;
    } mouse;
    /* keyboard key information */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t w : 1;
            uint16_t s : 1;
            uint16_t a : 1;
            uint16_t d : 1;
            uint16_t shift : 1;
            uint16_t ctrl : 1;
            uint16_t q : 1;
            uint16_t e : 1;
            uint16_t r : 1;
            uint16_t f : 1;
            uint16_t g : 1;
            uint16_t z : 1;
            uint16_t x : 1;
            uint16_t c : 1;
            uint16_t v : 1;
            uint16_t b : 1;
        } bit;
    } kb;
    int16_t wheel;
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

    // 遥控器DR16对外接口信息
    Dr16Data data_;

protected:
    // 初始化相关常量

    // 绑定的UART
    UartManageObject *uart_manage_obj_;

    // 常量

    // 内部变量

    // 当前时刻的遥控器DR16接收flag
    uint32_t rx_flag_ = 0;
    // 前一时刻的遥控器DR16接收flag
    uint32_t last_rx_flag_ = 0;

    // 读变量

    // 遥控器DR16状态
    Dr16Status dr16_status_ = DR16_STATUS_DISABLE;

    // 写变量

    // 读写变量

    // 内部函数

    void process_data(uint8_t *rx_data, uint16_t length);
};
/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
