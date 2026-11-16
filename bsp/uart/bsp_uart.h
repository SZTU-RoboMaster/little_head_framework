/**
 * @file bsp_uart.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-10 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
#define UART_BUFFER_SIZE 256

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*BspUart_Callback_t)(uint8_t *buffer, uint16_t length);

/**
 * @brief UART通信处理结构体
 */
typedef struct
{
    UART_HandleTypeDef *uart_handle;
    uint8_t tx_buffer[UART_BUFFER_SIZE];
    uint8_t rx_buffer[2][UART_BUFFER_SIZE];
    uint16_t rx_buffer_length;
    BspUart_Callback_t callback_func;
} BspUart_Manage_t;

/* Exported variables ---------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t initialized;

extern BspUart_Manage_t bsp_uart1;
extern BspUart_Manage_t bsp_uart3;
extern BspUart_Manage_t bsp_uart6;

/* Exported function declarations ---------------------------------------------*/

HAL_StatusTypeDef BspUart_MultiBufferReceiveToIdleDma(UART_HandleTypeDef *huart,
                                                       uint8_t *DstAddress,
                                                       uint8_t *SecondMemAddress,
                                                       uint16_t DataLength);

void BspUart_Init(UART_HandleTypeDef *huart, BspUart_Callback_t callback_func, uint16_t rx_buffer_length);

void BspUart_Reinit(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
