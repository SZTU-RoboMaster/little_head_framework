/**
 * @file shoot.h
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

#include "dr16.h"
#include "referee.h"
#include "motor_dji.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

struct ShootInput
{
    uint8_t sw_1;  // 发射控制
    uint8_t sw_2;  // 云台控制
    int16_t wheel; // 拨轮
    // 当前热量
    float current_ref_heat_ = 0.0f;
    // 热量限制上限
    float heat_limit_ = 0.0f;
    // 热量冷却速度
    float heat_cooling_rate_ = 0.0f;
};

struct ShootFeedback
{
    float trigger_angle;
    float trigger_omega;
    float trigger_current;
    float left_fric_omega;
    float right_fric_omega;
    float left_fric_current;
    float right_fric_current;
};

struct ShootOutput
{
    // 拨盘目标角度
    float target_trigger_angle = 0.0f;
    // 拨盘目标角速度
    float target_trigger_omega = 0.0f;
    // 拨盘目标电流
    float target_trigger_current = 0.0f;
    // 左摩擦轮目标角速度
    float target_left_fric_omega = 0.0f;
    // 右摩擦轮目标角速度
    float target_right_fric_omega = 0.0f;
    // 左摩擦轮目标电流
    float target_left_fric_current = 0.0f;
    // 右摩擦轮目标电流
    float target_right_fric_current = 0.0f;
};

enum ShootMode
{
    SHOOT_RELAX,
    SHOOT_IDLE,
    SHOOT_SINGLE,
    SHOOT_DOUBLE,
    SHOOT_TRIPLE,
    SHOOT_CONTINUE,
};

enum  FrictionState
{
    FRCTION_RELAX,
    FRCTION_IDLE,
    FRCTION_SUSPECT,
    FRCTION_CONFIRMED,
};

enum TriggerState
{
    TRIGGER_RELAX,
    TRIGGER_IDLE,
    TRIGGER_ANGLE,
    TRIGGER_SPEED,
    TRIGGER_BLOCK,
};

enum BlockState
{
    BLOCK_NORMAL,
    BLOCK_SUSPECT,
    BLOCK_CONFIRMED,
    BLOCK_PROCESSING,
};

/**
 * @brief Specialized, 发射机构类
 *
 */
class Shoot
{
public:
    // 遥控器
    Dr16 *dr16_;

    // 裁判系统
    Referee *referee_;

    // 拨弹盘电机
    MotorDji trigger_;

    // 摩擦轮电机左
    MotorDji friction_left_;

    // 摩擦轮电机右
    MotorDji friction_right_;

    void init();

    void update_input();

    void update_feedback();

    void handle_safety();

    void set_mode();

    void update_control_state();

    void control();

    void output();

protected:
    // 初始化相关常量

    // 常量

    // 堵转电流阈值
    float block_current_threshold_ = 9000.0f;
    // 堵转时间阈值
    uint16_t block_time_threshold_ = 500;
    // 堵转处理时间阈值
    uint16_t block_recovery_time_threshold_ = 500;

    // 内部变量

    // 摩擦轮flag
    uint8_t fric_enabled_ = false;
    // 单发flag
    uint8_t single_shot_pending_ = false;

    // 停火热量阈值
    float heat_ceasefire_threshold_ = 20.0f;
    // 减速热量阈值
    float heat_slowdown_threshold_ = 50.0f;

    // 读变量

    // 发射机构输入
    ShootInput input_;

    // 发射机构反馈
    ShootFeedback feedback_;

    // 发射机构状态
    ShootMode shoot_mode_;

    // 发射机构输出
    ShootOutput control_output_;

    // 摩擦轮状态
    FrictionState friction_state_;

    // 拨盘状态
    TriggerState trigger_state_;

    // 堵转状态
    BlockState block_state_;

    // 写变量

    // 当前热量
    float current_heat_ = 0.0f;

    // 读写变量

    // 摩擦轮角速度
    float fric_target_omega_ = 700.0f;

    // 内部函数
    void update_friction_state();

    void update_heat_state();

    void update_trigger_state();

    void update_block_state();
};

/* Exported variables ---------------------------------------------------------*/
extern Shoot shoot;

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
