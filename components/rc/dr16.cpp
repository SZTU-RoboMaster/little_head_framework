/**
 * @file dr16.cpp
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
#include <cstdlib>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 遥控器DR16初始化
 *
 * @param huart 指定的UART
 */
void Dr16::init(UART_HandleTypeDef *huart)
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

    dr16_publisher_ = MessageCenter::instance().advertise<Dr16Message>(kDr16TopicName);
}

/**
 * @brief UART通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Dr16::uart_rx_callback(uint8_t *rx_data, uint16_t length)
{
    // 半帧导致帧错位, 重启 DMA 让下一帧从缓冲区起始重新对齐
    if (length != uart_manage_obj_->rx_buffer_length)
    {
        uart_reinit(uart_manage_obj_->uart_handle);
        return;
    }

    // 滑动窗口, 判断遥控器DR16是否在线
    rx_flag_ += 1;

    process_data(rx_data, length);
    dr16_publisher_.publish(data_);
}

/**
 * @brief TIM定时器中断定期检测遥控器DR16是否存活
 *
 */
void Dr16::check_alive_100ms()
{
    // 判断该时间段内是否接收过遥控器DR16数据
    if (rx_flag_ == last_rx_flag_)
    {
        // 遥控器DR16断开连接
        dr16_status_ = DR16_STATUS_DISABLE;

        uart_reinit(uart_manage_obj_->uart_handle);
    }
    else
    {
        // 遥控器DR16保持连接
        dr16_status_ = DR16_STATUS_ENABLE;
    }
    last_rx_flag_ = rx_flag_;
}

/**
 * @brief 数据处理过程
 *
 */
void Dr16::process_data(uint8_t *rx_data, uint16_t length)
{

    data_.ch_0 = ((rx_data[0] | rx_data[1] << 8) & 0x07ff) - 1024;
    data_.ch_1 = ((rx_data[1] >> 3 | rx_data[2] << 5) & 0x07ff) - 1024;
    data_.ch_2 = ((rx_data[2] >> 6 | rx_data[3] << 2 | rx_data[4] << 10) & 0x07ff) - 1024;
    data_.ch_3 = ((rx_data[4] >> 1 | rx_data[5] << 7) & 0x07ff) - 1024;

    if ((std::abs(data_.ch_0) > 660) ||
        (std::abs(data_.ch_1) > 660) ||
        (std::abs(data_.ch_2) > 660) ||
        (std::abs(data_.ch_3) > 660))
    {
        data_ = {};
        return;
    }

    if (std::abs(data_.ch_0) <= 5) {data_.ch_0 = 0;}
    if (std::abs(data_.ch_1) <= 5) {data_.ch_1 = 0;}
    if (std::abs(data_.ch_2) <= 5) {data_.ch_2 = 0;}
    if (std::abs(data_.ch_3) <= 5) {data_.ch_3 = 0;}

    data_.sw_1 = ((rx_data[5] >> 4) & 0x000c) >> 2;
    data_.sw_2 = ((rx_data[5] >> 4) & 0x0003);

    data_.mouse.x = (rx_data[6] | rx_data[7] << 8);
    data_.mouse.y = (rx_data[8] | rx_data[9] << 8);
    data_.mouse.z = (rx_data[10] | rx_data[11] << 8);

    data_.mouse.left = rx_data[12];
    data_.mouse.right = rx_data[13];

    data_.kb.key_code = (rx_data[14] | rx_data[15] << 8);
    data_.wheel = (rx_data[16] | rx_data[17] << 8) - 1024;
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
