/**
 * @file vision.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-10 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "vision.h"

#include "CRC8_CRC16.h"
#include "bsp_usb.h"
#include "math_tools.h"
#include "protocol.h"

#include <cstring>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/
void Vision::init()
{
    vision_publisher_ = MessageCenter::instance().advertise<VisionMessage>(kVisionTopicName);
    ins_subscriber_ = MessageCenter::instance().subscribe<InsMessage>(kInsTopicName);
}

/**
 * @brief USB通信接收回调函数
 *
 * @param buf 接收的数据
 * @param len 数据长度
 */
void Vision::usb_rx_callback(uint8_t *buf, uint32_t len)
{
    update_rx(buf, len);
    publish();
}

void Vision::send()
{
    // 发送数据
    CDC_Transmit_FS(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
}

void Vision::publish()
{
    VisionMessage msg = {.yaw = rx_data_.yaw,
                         .yaw_vel = rx_data_.yaw_vel,
                         .yaw_acc = rx_data_.yaw_acc,
                         .pitch = rx_data_.pitch,
                         .pitch_vel = rx_data_.pitch_vel,
                         .pitch_acc = rx_data_.pitch_acc,
                         .target_lock = rx_data_.target_lock,
                         .fire_command = rx_data_.fire_command};

    vision_publisher_.publish(msg);
}

void Vision::update_tx()
{
    ins_subscriber_.update(ins_message_);

    tx_data_.header.sof = HEADER_SOF;
    tx_data_.header.data_length = sizeof(VisionData) - sizeof(FrameHeader) -
                                  sizeof(VisionData::cmd_id) - sizeof(VisionData::crc16) -
                                  sizeof(FrameTail);
    tx_data_.header.seq = seq_++;
    append_CRC8_check_sum(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(FrameHeader));
    tx_data_.cmd_id = VISION_ID;
    tx_data_.id = 7;
    tx_data_.mode = 33; // auto aim
    tx_data_.yaw = ins_message_.angle[2];
    tx_data_.yaw_vel = ins_message_.gyro[2];
    tx_data_.pitch = ins_message_.angle[0];
    tx_data_.pitch_vel = ins_message_.gyro[0];
    tx_data_.roll = ins_message_.angle[1];
    std::memcpy(tx_data_.quaternion, ins_message_.quaternion, sizeof(ins_message_.quaternion));
    tx_data_.shoot_speed = 23.0f;
    tx_data_.bullet_count = 100;
    tx_data_.game_progress = 1;
    append_CRC16_check_sum(reinterpret_cast<uint8_t *>(&tx_data_),
                           sizeof(VisionData) - sizeof(FrameTail));
    tx_data_.tail.end1 = END1_SOF;
    tx_data_.tail.end2 = END2_SOF;
}

/**
 * @brief
 *
 * @param

 */
void Vision::update_rx(uint8_t *buf, uint32_t len)
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

    while (rx_len_ >= 1)
    {
        if (rx_buffer_[0] == HEADER_SOF)
        {
            if (rx_len_ >= sizeof(RobotCtrlData))
            {
                if (verify_CRC16_check_sum(rx_buffer_, sizeof(RobotCtrlData) - sizeof(FrameTail)))
                {
                    std::copy(rx_buffer_, rx_buffer_ + sizeof(RobotCtrlData),
                              reinterpret_cast<uint8_t *>(&rx_data_));

                    // Shift remaining data forward
                    rx_len_ -= sizeof(RobotCtrlData);
                    if (rx_len_ > 0)
                    {
                        std::memmove(rx_buffer_, rx_buffer_ + sizeof(RobotCtrlData), rx_len_);
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
            // Invalid head, shift 1 byte forward to find next HEADER_SOF
            rx_len_--;
            if (rx_len_ > 0)
            {
                std::memmove(rx_buffer_, rx_buffer_ + 1, rx_len_);
            }
        }
    }
}
/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
