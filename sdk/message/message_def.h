/**
 * @file message_def.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-08-15 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/
static const char kInsTopicName[] = "/ins";
static const char kVisionTopicName[] = "/vision";
static const char kGimbalTopicName[] = "/gimbal";
static const char kDr16TopicName[] = "/dr16";
static const char kVt13TopicName[] = "/vt13";
static const char kCmdGimbalTopicName[] = "/command/gimbal";
static const char kCmdChassisTopicName[] = "/command/chassis";
static const char kCmdShootTopicName[] = "/command/shoot";
static const char kChassisTopicName[] = "/chassis";
static const char kPowerControllerTopicName[] = "/powercontroller";
static const char kRefereeTopicName[] = "/referee";

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    GIMBAL_CMD_RELAX,  // 云台失能
    GIMBAL_CMD_ACTIVE, // 云台使能
} GimbalCmdMode_e;

typedef enum
{
    CHASSIS_CMD_RELAX,  // 底盘失能
    CHASSIS_CMD_ONLY,   // 底盘独立
    CHASSIS_CMD_FOLLOW, // 底盘跟随
    CHASSIS_CMD_SPIN,   // 小陀螺
} ChassisCmdMode_e;

/**
 * @brief IMU数据
 * @brief ins发布
 * @brief gimbal vision订阅
 *
 */
typedef struct
{
    float angle[3];
    float gyro[3];
    float acc[3];
    float quaternion[4];
}Ins_Message_t;

/**
 * @brief 视觉自瞄数据
 * @brief vision发布
 * @brief gimbal订阅
 *
 */
typedef struct
{
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    int8_t target_lock; // 49: lock, 50: unlock
    int8_t fire_command;
}Vision_Message_t;

/**
 * @brief 云台数据
 * @brief gimbal发布
 * @brief chassis订阅
 *
 */
typedef struct
{
    float yaw_total_angle;
}Gimbal_Message_t;

/**
 * @brief 底盘控制数据
 * @brief command发布
 * @brief chassis订阅
 *
 */
typedef struct
{
    ChassisCmdMode_e chassis_mode;
    float rc_vx;
    float rc_vy;
    float rc_vw;
}Chassis_Cmd_Message_t;

/**
 * @brief 云台控制数据
 * @brief command发布
 * @brief gimbal订阅
 *
 */
typedef struct
{
    GimbalCmdMode_e gimbal_mode;
    float yaw_rate;
    float pitch_rate;
    uint8_t auto_aim;
}Gimbal_Cmd_Message_t;

/**
 * @brief 发射控制数据
 * @brief command发布
 * @brief shoot订阅
 *
 */
typedef struct
{
    uint8_t fric_enabled;
    uint8_t continue_shoot;
    uint32_t single_shot_seq;
    uint8_t auto_aim;
}Shoot_Cmd_Message_t;

/**
 * @brief 裁判系统反馈数据
 * @brief referee发布
 * @brief shoot powercontroller订阅
 *
 */
typedef struct
{
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_barrel_heat_limit;
    uint16_t shooter_barrel_cooling_value;

    uint16_t chassis_power_limit;
    uint16_t buffer_energy;
}Referee_Message_t;

/**
 * @brief 底盘数据
 * @brief chassis发布
 * @brief powercontroller订阅
 *
 */
typedef struct
{
    float feedback_omega[4];
    float cmd_omega[4];
    float cmd_current[4];
}Chassis_Message_t;

/**
 * @brief 功控数据
 * @brief powercontroller发布
 * @brief chassis订阅
 *
 */
typedef struct
{
    float target_current[4];
}Power_Controller_Message_t;

/**
 * @brief dr16数据
 * @brief dr16发布
 * @brief command订阅
 *
 */
typedef struct
{
    /* rocker channel information */
    int16_t ch_0;
    int16_t ch_1;
    int16_t ch_2;
    int16_t ch_3;
    /* left and right lever information */
    uint8_t sw_1; // 2: down, 3: mid, 1: up
    uint8_t sw_2; // 2: down, 3: mid, 1: up
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
}Dr16_Message_t;

/**
 * @brief vt13数据
 * @brief vt13发布
 * @brief command订阅
 *
 */
typedef struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    int16_t ch_0;
    int16_t ch_1;
    int16_t ch_2;
    int16_t ch_3;
    uint8_t mode_sw;
    uint8_t pause;
    uint8_t fn_1;
    uint8_t fn_2;
    int16_t wheel;
    uint8_t trigger;
    /* mouse movement and button information */
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t left;
        uint8_t right;
        uint8_t middle;
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
    uint16_t crc16;
}Vt13_Message_t;

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
