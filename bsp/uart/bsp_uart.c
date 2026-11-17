/**
 * @file bsp_uart.c
 * @author anchengc
 * @brief UART DMA空闲接收封装
 */

#include "bsp_uart.h"

BspUart_Manage_t bsp_uart1 = {0};
BspUart_Manage_t bsp_uart3 = {0};
BspUart_Manage_t bsp_uart5 = {0};

static BspUart_Manage_t *get_uart_manage(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        return &bsp_uart1;
    }
    if (huart->Instance == USART3)
    {
        return &bsp_uart3;
    }
    if (huart->Instance == UART5)
    {
        return &bsp_uart5;
    }
    return NULL;
}

static void start_receive(BspUart_Manage_t *manage)
{
    HAL_UARTEx_ReceiveToIdle_DMA(manage->uart_handle, manage->rx_buffer,
                                 manage->rx_buffer_length);
    __HAL_DMA_DISABLE_IT(manage->uart_handle->hdmarx, DMA_IT_HT);
}

void BspUart_Init(UART_HandleTypeDef *huart, BspUart_Callback_t callback_func,
                  uint16_t rx_buffer_length)
{
    BspUart_Manage_t *manage = get_uart_manage(huart);
    if (manage == NULL)
    {
        return;
    }

    manage->uart_handle = huart;
    manage->callback_func = callback_func;
    manage->rx_buffer_length = rx_buffer_length;
    start_receive(manage);
}

void BspUart_Reinit(UART_HandleTypeDef *huart)
{
    BspUart_Manage_t *manage = get_uart_manage(huart);
    if (manage == NULL)
    {
        return;
    }

    HAL_UART_DMAStop(huart);
    if (huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
        huart->hdmarx->State = HAL_DMA_STATE_READY;
        __HAL_UNLOCK(huart->hdmarx);
    }
    __HAL_UART_CLEAR_PEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    start_receive(manage);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (!initialized)
    {
        return;
    }

    BspUart_Manage_t *manage = get_uart_manage(huart);
    if (manage == NULL)
    {
        return;
    }

    if (manage->callback_func != NULL)
    {
        manage->callback_func(manage->rx_buffer, size);
    }
    start_receive(manage);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BspUart_Reinit(huart);
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
