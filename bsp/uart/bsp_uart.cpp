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
#define DATA_IN_D2_SRAM

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

__attribute__((section(".AXI_SRAM"), aligned(32))) UartManageObject uart1_manage_obj = {};
__attribute__((section(".AXI_SRAM"), aligned(32))) UartManageObject uart5_manage_obj = {};
__attribute__((section(".AXI_SRAM"), aligned(32))) UartManageObject uart7_manage_obj = {};
__attribute__((section(".AXI_SRAM"), aligned(32))) UartManageObject uart10_manage_obj = {};

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
HAL_StatusTypeDef UARTEx_MultiBuffer_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
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

        status = HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->RDR,
                                            (uint32_t)DstAddress, (uint32_t)SecondMemAddress,
                                            DataLength);
        if (status != HAL_OK)
        {
            huart->RxState = HAL_UART_STATE_READY;
            return status;
        }

        // 清除错误标志，因为在DMA启动前可能会有错误标志残留(我发现是ORE)，导致不断进入错误中断，无法接收数据
        __HAL_UART_CLEAR_FLAG(huart,
                              UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
        // 丢掉RQR寄存器中的脏数据
        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

        if (huart->Init.Parity != UART_PARITY_NONE)
        {
            ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);
        }
        ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_EIE | USART_CR3_DMAR);

        if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
        {
            __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
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
void uart_init(UART_HandleTypeDef *huart, uart_callback_t callback_func, uint16_t rx_buffer_length)
{
    if (huart->Instance == USART1)
    {
        uart1_manage_obj.uart_handle = huart;
        uart1_manage_obj.callback_func = callback_func;
        uart1_manage_obj.rx_buffer_length = rx_buffer_length;
        UARTEx_MultiBuffer_ReceiveToIdle_DMA(huart, uart1_manage_obj.rx_buffer_0,
                                             uart1_manage_obj.rx_buffer_1, rx_buffer_length);
    }
    else if (huart->Instance == UART5)
    {
        uart5_manage_obj.uart_handle = huart;
        uart5_manage_obj.callback_func = callback_func;
        uart5_manage_obj.rx_buffer_length = rx_buffer_length;
        UARTEx_MultiBuffer_ReceiveToIdle_DMA(huart, uart5_manage_obj.rx_buffer_0,
                                             uart5_manage_obj.rx_buffer_1, rx_buffer_length);
    }
    else if (huart->Instance == UART7)
    {
        uart7_manage_obj.uart_handle = huart;
        uart7_manage_obj.callback_func = callback_func;
        uart7_manage_obj.rx_buffer_length = rx_buffer_length;
        UARTEx_MultiBuffer_ReceiveToIdle_DMA(huart, uart7_manage_obj.rx_buffer_0,
                                             uart7_manage_obj.rx_buffer_1, rx_buffer_length);
    }
    else if (huart->Instance == USART10)
    {
        uart10_manage_obj.uart_handle = huart;
        uart10_manage_obj.callback_func = callback_func;
        uart10_manage_obj.rx_buffer_length = rx_buffer_length;
        UARTEx_MultiBuffer_ReceiveToIdle_DMA(huart, uart10_manage_obj.rx_buffer_0,
                                             uart10_manage_obj.rx_buffer_1, rx_buffer_length);
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
    if (huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
        huart->hdmarx->State = HAL_DMA_STATE_READY;
        __HAL_UNLOCK(huart->hdmarx);
    }

    // 清除错误标志及错误码, 避免一直进错误中断
    __HAL_UART_CLEAR_FLAG(huart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (huart->Instance == USART1)
    {
        UARTEx_MultiBuffer_ReceiveToIdle_DMA(huart, uart1_manage_obj.rx_buffer_0,
                                             uart1_manage_obj.rx_buffer_1,
                                             uart1_manage_obj.rx_buffer_length);
    }
    else if (huart->Instance == UART5)
    {
        UARTEx_MultiBuffer_ReceiveToIdle_DMA(huart, uart5_manage_obj.rx_buffer_0,
                                             uart5_manage_obj.rx_buffer_1,
                                             uart5_manage_obj.rx_buffer_length);
    }
    else if (huart->Instance == UART7)
    {
        UARTEx_MultiBuffer_ReceiveToIdle_DMA(huart, uart7_manage_obj.rx_buffer_0,
                                             uart7_manage_obj.rx_buffer_1,
                                             uart7_manage_obj.rx_buffer_length);
    }
    else if (huart->Instance == USART10)
    {
        UARTEx_MultiBuffer_ReceiveToIdle_DMA(huart, uart10_manage_obj.rx_buffer_0,
                                             uart10_manage_obj.rx_buffer_1,
                                             uart10_manage_obj.rx_buffer_length);
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
        if (uart1_manage_obj.callback_func == nullptr)
        {
            return;
        }

        if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
        {
            uart1_manage_obj.callback_func(uart1_manage_obj.rx_buffer_1, size);
        }
        else
        {
            uart1_manage_obj.callback_func(uart1_manage_obj.rx_buffer_0, size);
        }
    }
    else if (huart->Instance == UART5)
    {
        if (uart5_manage_obj.callback_func == nullptr)
        {
            return;
        }

        if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
        {
            uart5_manage_obj.callback_func(uart5_manage_obj.rx_buffer_1, size);
        }
        else
        {
            uart5_manage_obj.callback_func(uart5_manage_obj.rx_buffer_0, size);
        }
    }
    else if (huart->Instance == UART7)
    {
        if (uart7_manage_obj.callback_func == nullptr)
        {
            return;
        }

        if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
        {
            uart7_manage_obj.callback_func(uart7_manage_obj.rx_buffer_1, size);
        }
        else
        {
            uart7_manage_obj.callback_func(uart7_manage_obj.rx_buffer_0, size);
        }
    }
    else if (huart->Instance == USART10)
    {
        if (uart10_manage_obj.callback_func == nullptr)
        {
            return;
        }

        if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
        {
            uart10_manage_obj.callback_func(uart10_manage_obj.rx_buffer_1, size);
        }
        else
        {
            uart10_manage_obj.callback_func(uart10_manage_obj.rx_buffer_0, size);
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
    uart_reinit(huart);
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
