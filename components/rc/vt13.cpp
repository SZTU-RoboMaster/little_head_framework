/**
 * @file vt13.cpp
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
#include <cstdlib>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 遥控器VT13初始化
 *
 * @param huart 指定的UART
 */
void Vt13::init(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_manage_obj_ = &uart1_manage_obj;
    }
    else if (huart->Instance == USART3)
    {
        uart_manage_obj_ = &uart3_manage_obj;
    }
    else if (huart->Instance == USART6)
    {
        uart_manage_obj_ = &uart6_manage_obj;
    }
    publisher_ = MessageCenter::instance().advertise<Vt13Message>(kVt13TopicName);
}

/**
 * @brief UART通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Vt13::uart_rx_callback(uint8_t *rx_data, uint16_t length)
{
    // 半帧导致帧错位, 重启 DMA 让下一帧从缓冲区起始重新对齐
    if (length != uart_manage_obj_->rx_buffer_length)
    {
        uart_reinit(uart_manage_obj_->uart_handle);
        return;
    }

    // 滑动窗口, 判断遥控器VT13是否在线
    rx_flag_ += 1;

    process_data(rx_data, length);
    publisher_.publish(data_);
}

/**
 * @brief TIM定时器中断定期检测遥控器VT13是否存活
 *
 */
void Vt13::check_alive_100ms()
{
    // 判断该时间段内是否接收过遥控器VT13数据
    if (rx_flag_ == last_rx_flag_)
    {
        // 遥控器VT13断开连接
        vt13_status_ = VT13_STATUS_DISABLE;

        uart_reinit(uart_manage_obj_->uart_handle);
    }
    else
    {
        // 遥控器VT13保持连接
        vt13_status_ = VT13_STATUS_ENABLE;
    }
    last_rx_flag_ = rx_flag_;
}

/**
 * @brief 数据处理过程
 *
 */
void Vt13::process_data(uint8_t *rx_data, uint16_t length)
{
    // clang-format off
    data_.sof_1 = rx_data[0];
    data_.sof_2 = rx_data[1];
    if (data_.sof_1 != 0xA9 || data_.sof_2 != 0x53)
    {
        data_ = {.mode_sw = 0};
        return;
    }

    if (!verify_CRC16_check_sum(rx_data, length))
    {
        data_ = {.mode_sw = 0};
        return;
    }

    data_.ch_0 = ((rx_data[2] | rx_data[3] << 8) & 0x07ff) - 1024;
    data_.ch_1 = ((rx_data[3] >> 3 | rx_data[4] << 5) & 0x07ff) - 1024;
    data_.ch_2 = ((rx_data[4] >> 6 | rx_data[5] << 2 | rx_data[6] << 10) & 0x07ff) - 1024;
    data_.ch_3 = ((rx_data[6] >> 1 | rx_data[7] << 7) & 0x07ff) - 1024;

    if ((std::abs(data_.ch_0) > 660) ||
        (std::abs(data_.ch_1) > 660) ||
        (std::abs(data_.ch_2) > 660) ||
        (std::abs(data_.ch_3) > 660))
    {
        data_ = {.mode_sw = 0};
        return;
    }

    if (std::abs(data_.ch_0) <= 5) {data_.ch_0 = 0;}
    if (std::abs(data_.ch_1) <= 5) {data_.ch_1 = 0;}
    if (std::abs(data_.ch_2) <= 5) {data_.ch_2 = 0;}
    if (std::abs(data_.ch_3) <= 5) {data_.ch_3 = 0;}

    data_.mode_sw = (rx_data[7] >> 4) & 0x03;
    data_.pause = (rx_data[7] >> 6) & 0x01;

    data_.fn_1 = rx_data[7] >> 7 & 0x01;
    data_.fn_2 = rx_data[8] & 0x01;

    data_.wheel = ((rx_data[8] >> 1 | rx_data[9] << 7) & 0x07ff) - 1024;
    data_.trigger = (rx_data[9] >> 4) & 0x01;

    data_.mouse.x = (rx_data[10] | rx_data[11] << 8);
    data_.mouse.y = (rx_data[12] | rx_data[13] << 8);
    data_.mouse.z = (rx_data[14] | rx_data[15] << 8);

    data_.mouse.left = rx_data[16] & 0x03;
    data_.mouse.right = (rx_data[16] >> 2) & 0x03;
    data_.mouse.middle = (rx_data[16] >> 4) & 0x03;

    data_.kb.key_code = (rx_data[17] | rx_data[18] << 8);
    data_.crc16 = (rx_data[19] | rx_data[20] << 8);
    // clang-format on
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
