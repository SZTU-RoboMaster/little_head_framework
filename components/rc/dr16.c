/**
 * @file dr16.c
 * @author anchengc
 * @brief 遥控器DR16实现
 * @version 0.1
 * @date 2026-06-10 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "dr16.h"

#include "math_tools.h"
#include <stdlib.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Dr16_t dr16;

/* Private function declarations ---------------------------------------------*/
static void Dr16_ProcessData(Dr16_t *self, uint8_t *rx_data, uint16_t length);

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 遥控器DR16初始化
 *
 * @param huart 指定的UART
 */
void Dr16_Init(Dr16_t *self, UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        self->uart_manage_obj = &bsp_uart1;
    }
    else if (huart->Instance == USART3)
    {
        self->uart_manage_obj = &bsp_uart3;
    }
    else if (huart->Instance == UART5)
    {
        self->uart_manage_obj = &bsp_uart5;
    }

    Create_Publisher(&self->publisher, kDr16TopicName, sizeof(self->data));
}

/**
 * @brief UART通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Dr16_UartRxCallback(Dr16_t *self, uint8_t *rx_data, uint16_t length)
{
    // 半帧导致帧错位, 重启 DMA 让下一帧从缓冲区起始重新对齐
    if (length != self->uart_manage_obj->rx_buffer_length)
    {
        BspUart_Reinit(self->uart_manage_obj->uart_handle);
        return;
    }

    // 滑动窗口, 判断遥控器DR16是否在线
    self->rx_flag += 1;

    Dr16_ProcessData(self, rx_data, length);
    Message_PublishFromISR(&self->publisher, &self->data, sizeof(self->data));
}

/**
 * @brief TIM定时器中断定期检测遥控器DR16是否存活
 *
 */
void Dr16_CheckAlive100ms(Dr16_t *self)
{
    // 判断该时间段内是否接收过遥控器DR16数据
    if (self->rx_flag == self->last_rx_flag)
    {
        // 遥控器DR16断开连接
        self->status = DR16_STATUS_DISABLE;

        BspUart_Reinit(self->uart_manage_obj->uart_handle);
    }
    else
    {
        // 遥控器DR16保持连接
        self->status = DR16_STATUS_ENABLE;
    }
    self->last_rx_flag = self->rx_flag;
}

/**
 * @brief 数据处理过程
 *
 */
static void Dr16_ProcessData(Dr16_t *self, uint8_t *rx_data, uint16_t length)
{
    // clang-format off
    self->data.ch_0 = ((rx_data[0] | rx_data[1] << 8) & 0x07ff) - 1024;
    self->data.ch_1 = ((rx_data[1] >> 3 | rx_data[2] << 5) & 0x07ff) - 1024;
    self->data.ch_2 = ((rx_data[2] >> 6 | rx_data[3] << 2 | rx_data[4] << 10) & 0x07ff) - 1024;
    self->data.ch_3 = ((rx_data[4] >> 1 | rx_data[5] << 7) & 0x07ff) - 1024;

    if ((abs(self->data.ch_0) > 660) ||
        (abs(self->data.ch_1) > 660) ||
        (abs(self->data.ch_2) > 660) ||
        (abs(self->data.ch_3) > 660))
    {
        self->data = (Dr16_Message_t){.sw_2 = 2};
        return;
    }

    if (abs(self->data.ch_0) <= 5) {self->data.ch_0 = 0;}
    if (abs(self->data.ch_1) <= 5) {self->data.ch_1 = 0;}
    if (abs(self->data.ch_2) <= 5) {self->data.ch_2 = 0;}
    if (abs(self->data.ch_3) <= 5) {self->data.ch_3 = 0;}

    self->data.sw_1 = ((rx_data[5] >> 4) & 0x000c) >> 2;
    self->data.sw_2 = ((rx_data[5] >> 4) & 0x0003);

    self->data.mouse.x = (rx_data[6] | rx_data[7] << 8);
    self->data.mouse.y = (rx_data[8] | rx_data[9] << 8);
    self->data.mouse.z = (rx_data[10] | rx_data[11] << 8);

    self->data.mouse.left = rx_data[12];
    self->data.mouse.right = rx_data[13];

    self->data.kb.key_code = (rx_data[14] | rx_data[15] << 8);
    self->data.wheel = (rx_data[16] | rx_data[17] << 8) - 1024;
    // clang-format on
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
