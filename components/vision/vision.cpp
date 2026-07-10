/**
 * @file vision.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "vision.h"
#include "CRC8_CRC16.h"
#include <algorithm>
#include <cstring>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief USB通信接收回调函数
 *
 * @param buf 接收的数据
 * @param len 数据长度
 */
void Vision::usb_rx_callback(uint8_t *buf, uint32_t len)
{

    // 滑动窗口, 判断遥控器DR16是否在线
    rx_flag_ += 1;

    update(buf, len);
}

/**
 * @brief TIM定时器中断定期检测是否存活
 *
 */
void Vision::check_alive_100ms()
{
    // 判断该时间段内是否接收过数据
    if (rx_flag_ == last_rx_flag_)
    {
        // 视觉断开连接
        vision_status_ = VISION_STATUS_DISABLE;
    }
    else
    {
        // 视觉保持连接
        vision_status_ = VISION_STATUS_ENABLE;
    }
    last_rx_flag_ = rx_flag_;
}

void Vision::send()
{
    // 发送数据
    tx_data_.crc16 = get_CRC16_check_sum(reinterpret_cast<uint8_t *>(&tx_data_),
                                         sizeof(tx_data_) - sizeof(tx_data_.crc16), 0xffff);
    CDC_Transmit_FS(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
}

/**
 * @brief
 *
 * @param

 */
void Vision::update(uint8_t *buf, uint32_t len)
{
    if (len == 0 || buf == nullptr)
    {
        return;
    }

    // Append new data to the buffer, discard all if overflow
    if (rx_len_ + len > sizeof(rx_buffer_))
    {
        rx_len_ = 0;
    }
    std::copy(buf, buf + len, rx_buffer_ + rx_len_);
    rx_len_ += len;

    while (rx_len_ >= 2)
    {
        if (rx_buffer_[0] == 'H' && rx_buffer_[1] == 'J')
        {
            if (rx_len_ >= sizeof(VisionData))
            {
                if (verify_CRC16_check_sum(rx_buffer_, sizeof(VisionData)))
                {
                    std::copy(rx_buffer_, rx_buffer_ + sizeof(VisionData),
                              reinterpret_cast<uint8_t *>(&rx_data_));

                    // Shift remaining data forward
                    rx_len_ -= sizeof(VisionData);
                    if (rx_len_ > 0)
                    {
                        std::memmove(rx_buffer_, rx_buffer_ + sizeof(VisionData), rx_len_);
                    }
                }
                else
                {
                    // CRC failed, invalid frame, shift 1 byte forward to re-sync
                    rx_len_--;
                    if (rx_len_ > 0)
                    {
                        std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_);
                    }
                }
            }
            else
            {
                break; // Wait for more data
            }
        }
        else
        {
            // Invalid head, shift 1 byte forward to find next 'H'
            rx_len_--;
            if (rx_len_ > 0)
            {
                std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_);
            }
        }
    }
}
/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
