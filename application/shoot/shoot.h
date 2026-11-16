/**
 * @file shoot.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-07-11 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "motor_dji.h"
#include "message_center.h"
#include "message_def.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    float trigger_angle;
    float trigger_omega;
    float trigger_current;
    float left_fric_omega;
    float right_fric_omega;
    float left_fric_current;
    float right_fric_current;

    // 当前热量
    float current_ref_heat;
    // 热量限制上限
    float heat_limit;
    // 热量冷却速度
    float heat_cooling_rate;
} Shoot_Feedback_t;

typedef struct
{
    // 拨盘目标角度
    float target_trigger_angle;
    // 拨盘目标角速度
    float target_trigger_omega;
    // 拨盘目标电流
    float target_trigger_current;
    // 左摩擦轮目标角速度
    float target_left_fric_omega;
    // 右摩擦轮目标角速度
    float target_right_fric_omega;
    // 左摩擦轮目标电流
    float target_left_fric_current;
    // 右摩擦轮目标电流
    float target_right_fric_current;
} Shoot_Output_t;

typedef enum
{
    SHOOT_RELAX,
    SHOOT_IDLE,
    SHOOT_SINGLE,
    SHOOT_DOUBLE,
    SHOOT_TRIPLE,
    SHOOT_CONTINUE,
} ShootMode_e;

typedef enum
{
    FRICTION_RELAX,
    FRICTION_IDLE,
    FRICTION_SUSPECT,
    FRICTION_CONFIRMED,
} FrictionState_e;

typedef enum
{
    TRIGGER_RELAX,
    TRIGGER_IDLE,
    TRIGGER_ANGLE,
    TRIGGER_SPEED,
    TRIGGER_BLOCK,
} TriggerState_e;

typedef enum
{
    BLOCK_NORMAL,
    BLOCK_SUSPECT,
    BLOCK_CONFIRMED,
    BLOCK_PROCESSING,
} BlockState_e;

typedef struct
{
    // 堵转电流阈值
    float block_current_threshold;
    // 堵转时间阈值
    uint16_t block_time_threshold;
    // 堵转处理时间阈值
    uint16_t block_recovery_time_threshold;
    // 停火热量阈值
    float heat_ceasefire_threshold;
    // 减速热量阈值
    float heat_slowdown_threshold;
    // 摩擦轮角速度
    float fric_target_omega;
} Shoot_Config_t;

typedef struct
{
    ShootMode_e mode;
    FrictionState_e friction_state;
    TriggerState_e trigger_state;
    BlockState_e block_state;
} Shoot_Status_t;

/**
 * @brief 发射机构控制数据
 */
typedef struct
{
    // 拨弹盘电机
    MotorDji_t trigger;
    // 摩擦轮电机左
    MotorDji_t friction_left;
    // 摩擦轮电机右
    MotorDji_t friction_right;

    Shoot_Config_t config;
    Shoot_Feedback_t feedback;
    Shoot_Status_t status;
    Shoot_Output_t control_output;

    // 使能摩擦轮
    uint8_t fric_enabled;
    // 单发请求
    uint8_t single_shot_request;
    // 单发锁存
    uint8_t single_shot_pending;
    // 单发序号
    uint32_t last_single_shot_seq;
    // 视觉flag
    uint8_t auto_aim;
    // 当前热量
    float current_heat;
    // 原静态计数器
    uint16_t friction_count;
    uint16_t friction_temp_lost;
    uint16_t block_count;
    float last_ref_heat;
    uint8_t heat_first_in;

    // 订阅者
    Subscriber_t cmd_subscriber;
    Subscriber_t referee_subscriber;
    Subscriber_t vision_subscriber;
    // 数据
    Shoot_Cmd_Message_t cmd_msg;
    Referee_Message_t referee_msg;
    Vision_Message_t vision_msg;
} Shoot_t;

/* Exported variables ---------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern Shoot_t shoot;

/* Exported function declarations ---------------------------------------------*/
void Shoot_Init(void);
void Shoot_Update_Input(void);
void Shoot_Update_Feedback(void);
void Shoot_Handle_Safety(void);
void Shoot_Set_Mode(void);
void Shoot_Update_Control_State(void);
void Shoot_Control(void);
void Shoot_Output(void);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
