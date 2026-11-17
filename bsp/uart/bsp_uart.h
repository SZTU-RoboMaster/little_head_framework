/**
 * @file bsp_uart.h
 * @author anchengc
 * @brief UART DMA空闲接收封装
 */

#pragma once

#include "usart.h"

#define UART_BUFFER_SIZE 256

typedef void (*BspUart_Callback_t)(uint8_t *buffer, uint16_t length);

typedef struct
{
    UART_HandleTypeDef *uart_handle;
    uint8_t tx_buffer[UART_BUFFER_SIZE];
    uint8_t rx_buffer[UART_BUFFER_SIZE];
    uint16_t rx_buffer_length;
    BspUart_Callback_t callback_func;
} BspUart_Manage_t;

extern uint8_t initialized;
extern BspUart_Manage_t bsp_uart1;
extern BspUart_Manage_t bsp_uart3;
extern BspUart_Manage_t bsp_uart5;

void BspUart_Init(UART_HandleTypeDef *huart, BspUart_Callback_t callback_func,
                  uint16_t rx_buffer_length);
void BspUart_Reinit(UART_HandleTypeDef *huart);

/************************ COPYRIGHT(C) SZTU-HJ **************************/
