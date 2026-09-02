/**
 * @file vision.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-06-10 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "protocol.h"

#include "message_center.h"
#include "message_def.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Specialized
 *
 */
class Vision
{
public:
    void init();

    void usb_rx_callback(uint8_t *buf, uint32_t len);

    void send();

    void update_tx();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 接收缓冲区
    uint8_t rx_buffer_[512] = {0};
    // 接收字节长度
    uint16_t rx_len_ = 0;
    // 当前时刻的接收flag
    uint32_t rx_flag_ = 0;
    // 前一时刻的接收flag
    uint32_t last_rx_flag_ = 0;
    // 发送序列号
    uint8_t seq_ = 0;

    // 读变量
    VisionData tx_data_;
    Subscriber<InsMessage> ins_subscriber_;
    InsMessage ins_message_;

    // 写变量
    RobotCtrlData rx_data_;
    Publisher<VisionMessage> vision_publisher_;

    // 读写变量

    // 内部函数
    void update_rx(uint8_t *buf, uint32_t len);
    void publish();
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
