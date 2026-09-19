/**
 * @file bsp_uart.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-10 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

UartManageObject uart1_manage_obj = {0};
UartManageObject uart3_manage_obj = {0};
UartManageObject uart5_manage_obj = {0};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param callback_func 处理回调函数
 * @param rx_buffer_length 接收缓冲区长度
 */
void uart_init(UART_HandleTypeDef *huart, uart_callback_t callback_func, uint16_t rx_buffer_length)
{
    if (huart->Instance == USART1)
    {
        uart1_manage_obj.uart_handle = huart;
        uart1_manage_obj.callback_func = callback_func;
        uart1_manage_obj.rx_buffer_length = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart1_manage_obj.rx_buffer,
                                     uart1_manage_obj.rx_buffer_length);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == USART3)
    {
        uart3_manage_obj.uart_handle = huart;
        uart3_manage_obj.callback_func = callback_func;
        uart3_manage_obj.rx_buffer_length = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3_manage_obj.rx_buffer,
                                     uart3_manage_obj.rx_buffer_length);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == UART5)
    {
        uart5_manage_obj.uart_handle = huart;
        uart5_manage_obj.callback_func = callback_func;
        uart5_manage_obj.rx_buffer_length = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart5_manage_obj.rx_buffer,
                                     uart5_manage_obj.rx_buffer_length);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief 重新初始化UART
 *
 * @param huart UART编号
 */
void uart_reinit(UART_HandleTypeDef *huart)
{
    // 停止当前DMA接收并将RxState恢复为READY,否则重启会因状态非READY返回HAL_BUSY失败
    HAL_UART_DMAStop(huart);

    // HAL_UART_DMAStop内部的HAL_DMA_Abort仅在State==BUSY时才复位,
    // 这里强制关流并复位State/Lock起到保护
    if (huart->hdmarx != nullptr)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
        huart->hdmarx->State = HAL_DMA_STATE_READY;
        __HAL_UNLOCK(huart->hdmarx);
    }

    // 清除错误标志及错误码, 避免一直进错误中断
    __HAL_UART_CLEAR_PEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart1_manage_obj.rx_buffer,
                                     uart1_manage_obj.rx_buffer_length);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3_manage_obj.rx_buffer,
                                     uart3_manage_obj.rx_buffer_length);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart5_manage_obj.rx_buffer,
                                     uart5_manage_obj.rx_buffer_length);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief HAL库UART接收DMA空闲中断
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half
 * transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 * @note  ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
 *
 * @brief HAL库UART空闲中断回调函数
 * @param huart UART编号
 * @param size 长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (!initialized)
    {
        return;
    }

    if (huart->Instance == USART1)
    {
        if (uart1_manage_obj.callback_func != nullptr)
        {
            uart1_manage_obj.callback_func(uart1_manage_obj.rx_buffer, size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart1_manage_obj.rx_buffer,
                                     uart1_manage_obj.rx_buffer_length);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == USART3)
    {
        if (uart3_manage_obj.callback_func != nullptr)
        {
            uart3_manage_obj.callback_func(uart3_manage_obj.rx_buffer, size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3_manage_obj.rx_buffer,
                                     uart3_manage_obj.rx_buffer_length);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == UART5)
    {
        if (uart5_manage_obj.callback_func != nullptr)
        {
            uart5_manage_obj.callback_func(uart5_manage_obj.rx_buffer, size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart5_manage_obj.rx_buffer,
                                     uart5_manage_obj.rx_buffer_length);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief HAL库UART错误中断回调函数
 *
 * @param huart UART编号
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uart_reinit(huart);
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
