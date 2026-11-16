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
#ifdef __cplusplus
#include <cstdint>
// C++ keeps its default member values. C initializes values when creating an object.
#define MESSAGE_DEFAULT(value) = value
#else
#include <stdint.h>
#define MESSAGE_DEFAULT(value)
#endif

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
typedef enum GimbalCmdMode
{
    GIMBAL_CMD_RELAX,  // 云台失能
    GIMBAL_CMD_ACTIVE, // 云台使能
} GimbalCmdMode;

typedef enum ChassisCmdMode
{
    CHASSIS_CMD_RELAX,  // 底盘失能
    CHASSIS_CMD_ONLY,   // 底盘独立
    CHASSIS_CMD_FOLLOW, // 底盘跟随
    CHASSIS_CMD_SPIN,   // 小陀螺
} ChassisCmdMode;

/**
 * @brief IMU数据
 * @brief ins发布
 * @brief gimbal vision订阅
 *
 */
typedef struct InsMessage
{
    float angle[3];
    float gyro[3];
    float acc[3];
    float quaternion[4];
} InsMessage;

/**
 * @brief 视觉自瞄数据
 * @brief vision发布
 * @brief gimbal订阅
 *
 */
typedef struct VisionMessage
{
    float yaw MESSAGE_DEFAULT(0.0f);
    float yaw_vel MESSAGE_DEFAULT(0.0f);
    float yaw_acc MESSAGE_DEFAULT(0.0f);
    float pitch MESSAGE_DEFAULT(0.0f);
    float pitch_vel MESSAGE_DEFAULT(0.0f);
    float pitch_acc MESSAGE_DEFAULT(0.0f);
    int8_t target_lock MESSAGE_DEFAULT(50); // 49: lock, 50: unlock
    int8_t fire_command MESSAGE_DEFAULT(0);
} VisionMessage;

/**
 * @brief 云台数据
 * @brief gimbal发布
 * @brief chassis订阅
 *
 */
typedef struct GimbalMessage
{
    float yaw_total_angle MESSAGE_DEFAULT(0.0f);
} GimbalMessage;

/**
 * @brief 底盘控制数据
 * @brief command发布
 * @brief chassis订阅
 *
 */
typedef struct ChassisCmdMessage
{
    ChassisCmdMode chassis_mode MESSAGE_DEFAULT(CHASSIS_CMD_RELAX);
    float rc_vx MESSAGE_DEFAULT(0.0f);
    float rc_vy MESSAGE_DEFAULT(0.0f);
    float rc_vw MESSAGE_DEFAULT(0.0f);
} ChassisCmdMessage;

/**
 * @brief 云台控制数据
 * @brief command发布
 * @brief gimbal订阅
 *
 */
typedef struct GimbalCmdMessage
{
    GimbalCmdMode gimbal_mode MESSAGE_DEFAULT(GIMBAL_CMD_RELAX);
    float yaw_rate MESSAGE_DEFAULT(0.0f);
    float pitch_rate MESSAGE_DEFAULT(0.0f);
    uint8_t auto_aim MESSAGE_DEFAULT(0);
} GimbalCmdMessage;

/**
 * @brief 发射控制数据
 * @brief command发布
 * @brief shoot订阅
 *
 */
typedef struct ShootCmdMessage
{
    uint8_t fric_enabled MESSAGE_DEFAULT(0);
    uint8_t continue_shoot MESSAGE_DEFAULT(0);
    uint32_t single_shot_seq MESSAGE_DEFAULT(0);
    uint8_t auto_aim MESSAGE_DEFAULT(0);
} ShootCmdMessage;

/**
 * @brief 裁判系统反馈数据
 * @brief referee发布
 * @brief shoot powercontroller订阅
 *
 */
typedef struct RefereeMessage
{
    uint16_t shooter_17mm_barrel_heat MESSAGE_DEFAULT(0);
    uint16_t shooter_barrel_heat_limit MESSAGE_DEFAULT(65535);
    uint16_t shooter_barrel_cooling_value MESSAGE_DEFAULT(0);

    uint16_t chassis_power_limit MESSAGE_DEFAULT(50);
    uint16_t buffer_energy MESSAGE_DEFAULT(50);
} RefereeMessage;

/**
 * @brief 底盘数据
 * @brief chassis发布
 * @brief powercontroller订阅
 *
 */
typedef struct ChassisMessage
{
    float feedback_omega[4] MESSAGE_DEFAULT({0.0f});
    float cmd_omega[4] MESSAGE_DEFAULT({0.0f});
    float cmd_current[4] MESSAGE_DEFAULT({0.0f});
} ChassisMessage;

/**
 * @brief 功控数据
 * @brief powercontroller发布
 * @brief chassis订阅
 *
 */
typedef struct PowerControllerMessage
{
    float target_current[4] MESSAGE_DEFAULT({0.0f});
} PowerControllerMessage;

/**
 * @brief dr16数据
 * @brief dr16发布
 * @brief command订阅
 *
 */
typedef struct Dr16Message
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
} Dr16Message;

/**
 * @brief vt13数据
 * @brief vt13发布
 * @brief command订阅
 *
 */
typedef struct Vt13Message
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
} Vt13Message;

#undef MESSAGE_DEFAULT

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
