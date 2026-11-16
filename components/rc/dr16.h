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
typedef enum
{
    DR16_STATUS_DISABLE = 0,
    DR16_STATUS_ENABLE,
} Dr16Status_e;

/**
 * @brief Specialized, 遥控器DR16
 *
 */
typedef struct
{
    // 初始化相关常量

    // 绑定的UART
    BspUart_Manage_t *uart_manage_obj;

    // 常量

    // 内部变量

    // 当前时刻的遥控器接收flag
    uint32_t rx_flag;
    // 前一时刻的遥控器接收flag
    uint32_t last_rx_flag;

    // 读变量

    // 遥控器状态
    Dr16Status_e status;
    // 遥控器数据
    Dr16_Message_t data;

    // 写变量
    Publisher_t publisher;

    // 读写变量

    // 内部函数

} Dr16_t;
/* Exported variables ---------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern Dr16_t dr16;

/* Exported function declarations ---------------------------------------------*/
void Dr16_Init(Dr16_t *self, UART_HandleTypeDef *huart);
void Dr16_UartRxCallback(Dr16_t *self, uint8_t *rx_data, uint16_t length);
void Dr16_CheckAlive100ms(Dr16_t *self);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
