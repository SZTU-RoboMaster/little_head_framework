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
#include <cstdint>

/* Exported macros -----------------------------------------------------------*/
inline constexpr char kInsTopicName[] = "/ins";
inline constexpr char kVisionTopicName[] = "/vision";
inline constexpr char kGimbalTopicName[] = "/gimbal";
inline constexpr char kDr16TopicName[] = "/dr16";

/* Exported types ------------------------------------------------------------*/
/**
 * @brief IMU数据
 * @brief ins发布
 * @brief gimbal vision订阅
 * 
 */
struct InsMessage
{
    float angle[3];
    float gyro[3];
    float acc[3];
    float quaternion[4];
};

/**
 * @brief 视觉自瞄数据
 * @brief vision发布
 * @brief gimbal订阅
 * 
 */
struct VisionMessage
{
    float yaw = 0.0f;
    float yaw_vel = 0.0f;
    float yaw_acc = 0.0f;
    float pitch = 0.0f;
    float pitch_vel = 0.0f;
    float pitch_acc = 0.0f;
    uint8_t target_lock = 50; // 49: lock, 50: unlock
    uint8_t fire_command = 0;
};

/**
 * @brief 云台数据
 * @brief gimbal发布
 * @brief chassis订阅
 * 
 */
struct GimbalMessage
{
    float yaw_total_angle = 0.0f;
};

/**
 * @brief dr16数据
 * @brief dr16发布
 * @brief gimbal chassis订阅
 * 
 */
struct Dr16Message
{
    /* rocker channel information */
    int16_t ch_0;
    int16_t ch_1;
    int16_t ch_2;
    int16_t ch_3;
    /* left and right lever information */
    uint8_t sw_1;
    uint8_t sw_2;
    /* mouse movement and button information */
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t left;
        uint8_t right;
    } mouse;
    /* keyboard key information */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t w : 1;
            uint16_t s : 1;
            uint16_t a : 1;
            uint16_t d : 1;
            uint16_t shift : 1;
            uint16_t ctrl : 1;
            uint16_t q : 1;
            uint16_t e : 1;
            uint16_t r : 1;
            uint16_t f : 1;
            uint16_t g : 1;
            uint16_t z : 1;
            uint16_t x : 1;
            uint16_t c : 1;
            uint16_t v : 1;
            uint16_t b : 1;
        } bit;
    } kb;
    int16_t wheel;
};
/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
