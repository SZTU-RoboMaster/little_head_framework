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
typedef enum
{
    VT13_STATUS_DISABLE = 0,
    VT13_STATUS_ENABLE,
} Vt13Status_e;



/**
 * @brief Specialized, 遥控器VT13
 *
 */
typedef struct
{
    // 初始化相关常量

    // 绑定的UART
    BspUart_Manage_t *uart_manage_obj;

    // 常量

    // 内部变量

    // 当前时刻的遥控器VT13接收flag
    uint32_t rx_flag;
    // 前一时刻的遥控器VT13接收flag
    uint32_t last_rx_flag;

    // 读变量

    // 遥控器VT13状态
    Vt13Status_e status;
    // 遥控器VT13对外接口信息
    Vt13_Message_t data;

    // 写变量
    Publisher_t publisher;

    // 读写变量

    // 内部函数

} Vt13_t;
/* Exported variables ---------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern Vt13_t vt13;

/* Exported function declarations ---------------------------------------------*/
void Vt13_Init(Vt13_t *self, UART_HandleTypeDef *huart);
void Vt13_UartRxCallback(Vt13_t *self, uint8_t *rx_data, uint16_t length);
void Vt13_CheckAlive100ms(Vt13_t *self);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
