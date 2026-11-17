/**
 * @file protocol.h
 * @author anchengc
 * @brief 视觉串口协议
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/
#define HEADER_SOF 0xA5U
#define END1_SOF 0x0DU
#define END2_SOF 0x0AU
#define CTRL_CMD_ID 0x0102U
#define VISION_ID 0x0105U

/* Exported types ------------------------------------------------------------*/
typedef struct __attribute__((packed))
{
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
} FrameHeader;

typedef struct __attribute__((packed))
{
    uint8_t end1;
    uint8_t end2;
} FrameTail;

typedef struct __attribute__((packed))
{
    FrameHeader header;
    uint16_t cmd_id;
    uint16_t id;
    uint16_t mode; // 33: auto aim
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float roll;
    float quaternion[4]; // w, x, y, z
    float shoot_speed;
    uint16_t bullet_count;
    uint8_t game_progress;
    uint16_t crc16;
    FrameTail tail;
} VisionData;

typedef struct __attribute__((packed))
{
    FrameHeader header;
    uint16_t cmd_id;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    int8_t target_lock; // 49: lock, 50: unlock
    int8_t fire_command;
    uint16_t crc16;
    FrameTail tail;
} RobotCtrlData;

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
