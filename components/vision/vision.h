/**
 * @file template.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "bsp_usb.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 视觉自瞄状态
 *
 */
enum VisionStatus
{
    VISION_STATUS_DISABLE = 0,
    VISION_STATUS_ENABLE,
};

struct __attribute__((packed)) RobotData
{
    uint8_t head[2] = {'H', 'J'};
    uint8_t mode = 33; // 33: auto aim, 34: small buff, 35: big buff
    float yaw = 0.0f;
    float yaw_vel = 0.0f;
    float pitch = 0.0f;
    float pitch_vel = 0.0f;
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
    float shoot_speed = 0.0f;
    uint16_t bullet_count = 0;
    uint16_t crc16 = 0;
};

struct __attribute__((packed)) VisionData
{
    uint8_t head[2] = {'H', 'J'};
    float yaw = 0.0f;
    float yaw_vel = 0.0f;
    float yaw_acc = 0.0f;
    float pitch = 0.0f;
    float pitch_vel = 0.0f;
    float pitch_acc = 0.0f;
    uint8_t target_lock = 50; // 49: lock, 50: unlock
    uint8_t fire_command = 0;
    // uint8_t target = 0; // 0: empty, 1: hero, 2: engineer, 3: infantry3, 4: infantry4, 5:infantry5,
                        // 6: empty, 7: sentry, 8: outpost, 9: base
    uint16_t crc16 = 0;
};

/**
 * @brief Specialized
 *
 */
class Vision
{
public:

    RobotData tx_data_;

    VisionData rx_data_;

    VisionStatus vision_status_ = VISION_STATUS_DISABLE;

    void usb_rx_callback(uint8_t *buf, uint32_t len);

    void check_alive_100ms();

    void send();

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

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数

    void update(uint8_t *buf, uint32_t len);
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
