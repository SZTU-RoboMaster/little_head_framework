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
 * @brief  remote control information
 */
struct Vt13Data
{
    uint8_t sof_1;
    uint8_t sof_2;
    int16_t ch_0;
    int16_t ch_1;
    int16_t ch_2;
    int16_t ch_3;
    uint8_t mode_sw;
    uint8_t pause;
    uint8_t fn_1;
    uint8_t fn_2;
    int16_t wheel;
    uint8_t trigger;
    /* mouse movement and button information */
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t left;
        uint8_t right;
        uint8_t middle;
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
    uint16_t crc16;
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
    Vt13Data data_;

    // 写变量

    // 读写变量

    // 内部函数

    void process_data(uint8_t *rx_data, uint16_t length);
};
/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
