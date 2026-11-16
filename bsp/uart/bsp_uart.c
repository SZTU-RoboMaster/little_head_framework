/**
 * @file bsp_uart.c
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

BspUart_Manage_t bsp_uart1 = {0};
BspUart_Manage_t bsp_uart3 = {0};
BspUart_Manage_t bsp_uart6 = {0};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief DMA双缓冲初始化函数
 * @param huart UART编号
 * @param DstAddress 双缓冲区1地址
 * @param SecondMemAddress 双缓冲区2地址
 * @param DataLength 数据长度
 * @note  仿照 HAL_UARTEx_ReceiveToIdle_DMA 进行封装
 */
HAL_StatusTypeDef BspUart_MultiBufferReceiveToIdleDma(UART_HandleTypeDef *huart,
                                                       uint8_t *DstAddress,
                                                       uint8_t *SecondMemAddress,
                                                       uint16_t DataLength)
{
    HAL_StatusTypeDef status;
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        if ((DstAddress == NULL) || (SecondMemAddress == NULL) || (DataLength == 0U))
        {
            return HAL_ERROR;
        }

        huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
        huart->RxEventType = HAL_UART_RXEVENT_TC;
        huart->RxXferSize = DataLength;

        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_BUSY_RX;

        status = HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->DR,
                                            (uint32_t)DstAddress, (uint32_t)SecondMemAddress,
                                            DataLength);
        if (status != HAL_OK)
        {
            huart->RxState = HAL_UART_STATE_READY;
            return status;
        }

        ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

        if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);
            ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
        }
        else
        {
            huart->RxState = HAL_UART_STATE_READY;
            status = HAL_ERROR;
        }
        return status;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param callback_func 处理回调函数
 * @param rx_buffer_length 接收缓冲区长度
 */
void BspUart_Init(UART_HandleTypeDef *huart, BspUart_Callback_t callback_func, uint16_t rx_buffer_length)
{
    if (huart->Instance == USART1)
    {
        bsp_uart1.uart_handle = huart;
        bsp_uart1.callback_func = callback_func;
        bsp_uart1.rx_buffer_length = rx_buffer_length;
        BspUart_MultiBufferReceiveToIdleDma(huart, bsp_uart1.rx_buffer[0],
                                             bsp_uart1.rx_buffer[1], rx_buffer_length);
    }
    else if (huart->Instance == USART3)
    {
        bsp_uart3.uart_handle = huart;
        bsp_uart3.callback_func = callback_func;
        bsp_uart3.rx_buffer_length = rx_buffer_length;
        BspUart_MultiBufferReceiveToIdleDma(huart, bsp_uart3.rx_buffer[0],
                                             bsp_uart3.rx_buffer[1], rx_buffer_length);
    }
    else if (huart->Instance == USART6)
    {
        bsp_uart6.uart_handle = huart;
        bsp_uart6.callback_func = callback_func;
        bsp_uart6.rx_buffer_length = rx_buffer_length;
        BspUart_MultiBufferReceiveToIdleDma(huart, bsp_uart6.rx_buffer[0],
                                             bsp_uart6.rx_buffer[1], rx_buffer_length);
    }
}

/**
 * @brief 重新初始化UART
 *
 * @param huart UART编号
 */
void BspUart_Reinit(UART_HandleTypeDef *huart)
{
    // 停止当前DMA接收并将RxState恢复为READY,否则重启会因状态非READY返回HAL_BUSY失败
    HAL_UART_DMAStop(huart);

    // HAL_UART_DMAStop内部的HAL_DMA_Abort仅在State==BUSY时才复位,
    // 这里强制关流并复位State/Lock起到保护
    if (huart->hdmarx != NULL)
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
        BspUart_MultiBufferReceiveToIdleDma(huart, bsp_uart1.rx_buffer[0],
                                             bsp_uart1.rx_buffer[1],
                                             bsp_uart1.rx_buffer_length);
    }
    else if (huart->Instance == USART3)
    {
        BspUart_MultiBufferReceiveToIdleDma(huart, bsp_uart3.rx_buffer[0],
                                             bsp_uart3.rx_buffer[1],
                                             bsp_uart3.rx_buffer_length);
    }
    else if (huart->Instance == USART6)
    {
        BspUart_MultiBufferReceiveToIdleDma(huart, bsp_uart6.rx_buffer[0],
                                             bsp_uart6.rx_buffer[1],
                                             bsp_uart6.rx_buffer_length);
    }
}
/**
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
        if (bsp_uart1.callback_func == NULL)
        {
            return;
        }

        if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
        {
            bsp_uart1.callback_func(bsp_uart1.rx_buffer[1], size);
        }
        else
        {
            bsp_uart1.callback_func(bsp_uart1.rx_buffer[0], size);
        }
    }
    else if (huart->Instance == USART3)
    {
        if (bsp_uart3.callback_func == NULL)
        {
            return;
        }

        if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
        {
            bsp_uart3.callback_func(bsp_uart3.rx_buffer[1], size);
        }
        else
        {
            bsp_uart3.callback_func(bsp_uart3.rx_buffer[0], size);
        }
    }
    else if (huart->Instance == USART6)
    {
        if (bsp_uart6.callback_func == NULL)
        {
            return;
        }

        if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
        {
            bsp_uart6.callback_func(bsp_uart6.rx_buffer[1], size);
        }
        else
        {
            bsp_uart6.callback_func(bsp_uart6.rx_buffer[0], size);
        }
    }
}

/**
 * @brief HAL库UART错误中断回调函数
 *
 * @param huart UART编号
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BspUart_Reinit(huart);
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
