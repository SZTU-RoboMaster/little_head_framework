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
static constexpr char kInsTopicName[] = "/ins";
static constexpr char kVisionTopicName[] = "/vision";
static constexpr char kGimbalTopicName[] = "/gimbal";
static constexpr char kDr16TopicName[] = "/dr16";
static constexpr char kVt13TopicName[] = "/vt13";
static constexpr char kCmdGimbalTopicName[] = "/command/gimbal";
static constexpr char kCmdChassisTopicName[] = "/command/chassis";
static constexpr char kCmdShootTopicName[] = "/command/shoot";
static constexpr char kChassisTopicName[] = "/chassis";
static constexpr char kPowerControllerTopicName[] = "/powercontroller";
static constexpr char kRefereeTopicName[] = "/referee";

/* Exported types ------------------------------------------------------------*/
enum GimbalCmdMode
{
    GIMBAL_CMD_RELAX,  // 云台失能
    GIMBAL_CMD_ACTIVE, // 云台使能
};

enum ChassisCmdMode
{
    CHASSIS_CMD_RELAX,  // 底盘失能
    CHASSIS_CMD_ONLY,   // 底盘独立
    CHASSIS_CMD_FOLLOW, // 底盘跟随
    CHASSIS_CMD_SPIN,   // 小陀螺
};

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
 * @brief 底盘控制数据
 * @brief command发布
 * @brief chassis订阅
 *
 */
struct ChassisCmdMessage
{
    ChassisCmdMode chassis_mode = CHASSIS_CMD_RELAX;
    float rc_vx = 0.0f;
    float rc_vy = 0.0f;
    float rc_vw = 0.0f;
};

/**
 * @brief 云台控制数据
 * @brief command发布
 * @brief gimbal订阅
 *
 */
struct GimbalCmdMessage
{
    GimbalCmdMode gimbal_mode = GIMBAL_CMD_RELAX;
    float yaw_rate = 0.0f;
    float pitch_rate = 0.0f;
};

/**
 * @brief 发射控制数据
 * @brief command发布
 * @brief shoot订阅
 *
 */
struct ShootCmdMessage
{
    uint8_t fric_enabled = 0;
    uint8_t continue_shoot = 0;
    uint32_t single_shot_seq = 0;
};

/**
 * @brief 裁判系统反馈数据
 * @brief referee发布
 * @brief shoot powercontroller订阅
 *
 */
struct RefereeMessage
{
    uint16_t shooter_17mm_barrel_heat = 0;
    uint16_t shooter_barrel_heat_limit = 65535;
    uint16_t shooter_barrel_cooling_value = 0;

    uint16_t chassis_power_limit = 50;
    uint16_t buffer_energy = 50;
};

/**
 * @brief 底盘数据
 * @brief chassis发布
 * @brief powercontroller订阅
 *
 */
struct ChassisMessage
{
    float feedback_omega[4]{0.0f};
    float cmd_omega[4]{0.0f};
    float cmd_current[4]{0.0f};
};

/**
 * @brief 功控数据
 * @brief powercontroller发布
 * @brief chassis订阅
 *
 */
struct PowerControllerMessage
{
    float target_current[4]{0.0f};
};

/**
 * @brief dr16数据
 * @brief dr16发布
 * @brief command订阅
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
};

/**
 * @brief vt13数据
 * @brief vt13发布
 * @brief command订阅
 *
 */
struct Vt13Message
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
};

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
