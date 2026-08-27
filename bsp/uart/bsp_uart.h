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

#include "stm32f4xx_hal.h"
#include "usart.h"

/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
#define UART_BUFFER_SIZE 256

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*uart_callback_t)(uint8_t *buffer, uint16_t length);

/**
 * @brief UART通信处理结构体
 */
struct UartManageObject
{
    UART_HandleTypeDef *uart_handle;
    uint8_t tx_buffer[UART_BUFFER_SIZE];
    uint8_t rx_buffer[2][UART_BUFFER_SIZE];
    uint16_t rx_buffer_length;
    uart_callback_t callback_func;
};

/* Exported variables ---------------------------------------------------------*/
extern uint8_t initialized;

extern UartManageObject uart1_manage_obj;
extern UartManageObject uart3_manage_obj;
extern UartManageObject uart6_manage_obj;

/* Exported function declarations ---------------------------------------------*/

HAL_StatusTypeDef UARTEx_MultiBuffer_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                                       uint8_t *DstAddress,
                                                       uint8_t *SecondMemAddress,
                                                       uint16_t DataLength);

void uart_init(UART_HandleTypeDef *huart, uart_callback_t callback_func, uint16_t rx_buffer_length);

void uart_reinit(UART_HandleTypeDef *huart);

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
