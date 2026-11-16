/**
 * @file vt13.c
 * @author anchengc
 * @brief 遥控器VT13实现
 * @version 0.1
 * @date 2026-06-10 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "vt13.h"

#include "CRC8_CRC16.h"
#include "math_tools.h"
#include <stdlib.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Vt13_t vt13;

/* Private function declarations ---------------------------------------------*/
static void Vt13_ProcessData(Vt13_t *self, uint8_t *rx_data, uint16_t length);

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 遥控器VT13初始化
 *
 * @param huart 指定的UART
 */
void Vt13_Init(Vt13_t *self, UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        self->uart_manage_obj = &bsp_uart1;
    }
    else if (huart->Instance == USART3)
    {
        self->uart_manage_obj = &bsp_uart3;
    }
    else if (huart->Instance == USART6)
    {
        self->uart_manage_obj = &bsp_uart6;
    }
    Create_Publisher(&self->publisher, kVt13TopicName, sizeof(self->data));
}

/**
 * @brief UART通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Vt13_UartRxCallback(Vt13_t *self, uint8_t *rx_data, uint16_t length)
{
    // 半帧导致帧错位, 重启 DMA 让下一帧从缓冲区起始重新对齐
    if (length != self->uart_manage_obj->rx_buffer_length)
    {
        BspUart_Reinit(self->uart_manage_obj->uart_handle);
        return;
    }

    // 滑动窗口, 判断遥控器VT13是否在线
    self->rx_flag += 1;

    Vt13_ProcessData(self, rx_data, length);
    Message_PublishFromISR(&self->publisher, &self->data, sizeof(self->data));
}

/**
 * @brief TIM定时器中断定期检测遥控器VT13是否存活
 *
 */
void Vt13_CheckAlive100ms(Vt13_t *self)
{
    // 判断该时间段内是否接收过遥控器VT13数据
    if (self->rx_flag == self->last_rx_flag)
    {
        // 遥控器VT13断开连接
        self->status = VT13_STATUS_DISABLE;

        BspUart_Reinit(self->uart_manage_obj->uart_handle);
    }
    else
    {
        // 遥控器VT13保持连接
        self->status = VT13_STATUS_ENABLE;
    }
    self->last_rx_flag = self->rx_flag;
}

/**
 * @brief 数据处理过程
 *
 */
static void Vt13_ProcessData(Vt13_t *self, uint8_t *rx_data, uint16_t length)
{
    // clang-format off
    self->data.sof_1 = rx_data[0];
    self->data.sof_2 = rx_data[1];
    if (self->data.sof_1 != 0xA9 || self->data.sof_2 != 0x53)
    {
        self->data = (Vt13_Message_t){.mode_sw = 0};
        return;
    }

    if (!verify_CRC16_check_sum(rx_data, length))
    {
        self->data = (Vt13_Message_t){.mode_sw = 0};
        return;
    }

    self->data.ch_0 = ((rx_data[2] | rx_data[3] << 8) & 0x07ff) - 1024;
    self->data.ch_1 = ((rx_data[3] >> 3 | rx_data[4] << 5) & 0x07ff) - 1024;
    self->data.ch_2 = ((rx_data[4] >> 6 | rx_data[5] << 2 | rx_data[6] << 10) & 0x07ff) - 1024;
    self->data.ch_3 = ((rx_data[6] >> 1 | rx_data[7] << 7) & 0x07ff) - 1024;

    if ((abs(self->data.ch_0) > 660) ||
        (abs(self->data.ch_1) > 660) ||
        (abs(self->data.ch_2) > 660) ||
        (abs(self->data.ch_3) > 660))
    {
        self->data = (Vt13_Message_t){.mode_sw = 0};
        return;
    }

    if (abs(self->data.ch_0) <= 5) {self->data.ch_0 = 0;}
    if (abs(self->data.ch_1) <= 5) {self->data.ch_1 = 0;}
    if (abs(self->data.ch_2) <= 5) {self->data.ch_2 = 0;}
    if (abs(self->data.ch_3) <= 5) {self->data.ch_3 = 0;}

    self->data.mode_sw = (rx_data[7] >> 4) & 0x03;
    self->data.pause = (rx_data[7] >> 6) & 0x01;

    self->data.fn_1 = rx_data[7] >> 7 & 0x01;
    self->data.fn_2 = rx_data[8] & 0x01;

    self->data.wheel = ((rx_data[8] >> 1 | rx_data[9] << 7) & 0x07ff) - 1024;
    self->data.trigger = (rx_data[9] >> 4) & 0x01;

    self->data.mouse.x = (rx_data[10] | rx_data[11] << 8);
    self->data.mouse.y = (rx_data[12] | rx_data[13] << 8);
    self->data.mouse.z = (rx_data[14] | rx_data[15] << 8);

    self->data.mouse.left = rx_data[16] & 0x03;
    self->data.mouse.right = (rx_data[16] >> 2) & 0x03;
    self->data.mouse.middle = (rx_data[16] >> 4) & 0x03;

    self->data.kb.key_code = (rx_data[17] | rx_data[18] << 8);
    self->data.crc16 = (rx_data[19] | rx_data[20] << 8);
    // clang-format on
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
