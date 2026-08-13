/**
 * @file template.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "shoot.h"
#include "motor_dji.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Shoot shoot;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/
/**
 * @brief
 *
 * @param

 */
void Shoot::init()
{

    // 拨弹盘电机初始化
    trigger_.angle_pid_.init(15.0f, 0.0f, 0.0f, 0.0f, 0.0f, 48.0f, 0.005f);
    trigger_.omega_pid_.init(1200.0f, 60000.0f, 0.0f, 0.0f, 5000.0f, 10000.0f);
    trigger_.init(&hcan2, 0x200, 0x201, MOTOR_DJI_CONTROL_METHOD_ANGLE, 36.0f);

    // 摩擦轮电机初始化
    friction_left_.omega_pid_.init(40.0f, 0.0f, 0.0f, 0.0f, 0.0f, 16384.0f);
    friction_left_.init(&hcan2, 0x200, 0x202, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    friction_right_.omega_pid_.init(40.0f, 0.0f, 0.0f, 0.0f, 0.0f, 16384.0f);
    friction_right_.init(&hcan2, 0x200, 0x203, MOTOR_DJI_CONTROL_METHOD_OMEGA);

    dr16_subscriber_ = MessageCenter::instance().subscribe<Dr16Message>(kDr16TopicName);
}

uint8_t sw_1_up, wheel_up;
void Shoot::update_input()
{
    dr16_subscriber_.update(dr16_message_);

    input_.sw_1 = dr16_message_.sw_1;
    input_.sw_2 = dr16_message_.sw_2;
    input_.wheel = dr16_message_.wheel;
    input_.current_ref_heat_ = referee_->power_heat_data_.shooter_17mm_barrel_heat;
    input_.heat_limit_ = referee_->robot_state_.shooter_barrel_heat_limit;
    input_.heat_cooling_rate_ = referee_->robot_state_.shooter_barrel_cooling_value;

    //////////////////////////////////////////////////////////////////////////////////////////
    // 临时写个上升沿，后面写到遥控器里
    static uint8_t last_sw_1, mod20 = 0;
    sw_1_up = (last_sw_1 == 3 && input_.sw_1 == 1) ? 1 : 0;
    last_sw_1 = input_.sw_1;

    static uint16_t last_wheel = 0;
    wheel_up = (last_wheel < 300 && input_.wheel > 300) ? 1 : 0;
    if (mod20++ >= 20)
    {
        mod20 = 0;
        last_wheel = input_.wheel;
    }

    // 上升沿切换摩擦轮flag
    if (sw_1_up)
    {
        fric_enabled_ = !fric_enabled_;
    }
    //////////////////////////////////////////////////////////////////////////////////////////
}

void Shoot::update_feedback()
{
    feedback_.trigger_angle = trigger_.rx_data_.total_angle;
    feedback_.trigger_omega = trigger_.rx_data_.omega;
    feedback_.trigger_current = trigger_.rx_data_.current;
    feedback_.left_fric_omega = friction_left_.rx_data_.omega;
    feedback_.right_fric_omega = friction_right_.rx_data_.omega;
    feedback_.left_fric_current = friction_left_.rx_data_.current;
    feedback_.right_fric_current = friction_right_.rx_data_.current;
}

void Shoot::handle_safety()
{
    if (!dr16_subscriber_.is_fresh(100))
    {
        dr16_message_ = {};
        fric_enabled_ = false;
    }
}

void Shoot::set_mode()
{

    if (!fric_enabled_)
    {
        shoot_mode_ = SHOOT_RELAX;
        return;
    }

    switch (shoot_mode_)
    {
    case SHOOT_RELAX:
        shoot_mode_ = SHOOT_IDLE;
        break;
    case SHOOT_IDLE:
        if (input_.sw_1 == 2)
        {
            shoot_mode_ = SHOOT_CONTINUE;
            return;
        }
        if (wheel_up)
        {
            shoot_mode_ = SHOOT_SINGLE;
            single_shot_pending_ = true;
            return;
        }
        break;
    case SHOOT_SINGLE:
    case SHOOT_DOUBLE:
    case SHOOT_TRIPLE:
        if (abs(feedback_.trigger_angle - control_output_.target_trigger_angle) < 0.002f)
        {
            shoot_mode_ = SHOOT_IDLE;
            return;
        }
        break;
    case SHOOT_CONTINUE:
        if (input_.sw_1 != 2)
        {
            shoot_mode_ = SHOOT_IDLE;
            return;
        }
        break;
    }
}

void Shoot::update_control_state()
{
    update_friction_state();
    update_heat_state();
    update_block_state();
    update_trigger_state();
}

void Shoot::control()
{

    // 摩擦轮控制
    if (!fric_enabled_)
    {
        control_output_.target_left_fric_omega = 0.0f;
        control_output_.target_right_fric_omega = 0.0f;
    }
    else
    {
        control_output_.target_left_fric_omega = -fric_target_omega_;
        control_output_.target_right_fric_omega = fric_target_omega_;
    }

    // 拨盘控制
    switch (trigger_state_)
    {
    case TRIGGER_RELAX:
        trigger_.set_control_method(MOTOR_DJI_CONTROL_METHOD_CURRENT);
        control_output_.target_trigger_current = 0.0f;
        control_output_.target_trigger_angle = feedback_.trigger_angle;

        break;
    case TRIGGER_IDLE:
        trigger_.set_control_method(MOTOR_DJI_CONTROL_METHOD_OMEGA);
        control_output_.target_trigger_omega = 0.0f;
        control_output_.target_trigger_angle = feedback_.trigger_angle;

        break;
    case TRIGGER_ANGLE:
        trigger_.set_control_method(MOTOR_DJI_CONTROL_METHOD_ANGLE);

        if (single_shot_pending_)
        {
            control_output_.target_trigger_angle -= 2.0f * PI / 8.0f;
            single_shot_pending_ = false;
        }
        break;

    case TRIGGER_SPEED:
        trigger_.set_control_method(MOTOR_DJI_CONTROL_METHOD_OMEGA);
        control_output_.target_trigger_omega = -10.0f;

        break;
    case TRIGGER_BLOCK:
        trigger_.set_control_method(MOTOR_DJI_CONTROL_METHOD_OMEGA);

        control_output_.target_trigger_omega = 15.0f;

        break;
    }
}

void Shoot::output()
{
    trigger_.set_target_angle(control_output_.target_trigger_angle);
    trigger_.set_target_omega(control_output_.target_trigger_omega);
    trigger_.set_target_current(control_output_.target_trigger_current);

    friction_left_.set_target_omega(control_output_.target_left_fric_omega);

    friction_right_.set_target_omega(control_output_.target_right_fric_omega);
}

void Shoot::update_friction_state()
{
    // TODO: templost不是最优解，考虑滤波或均值
    static uint16_t cnt, templost = 0;

    cnt++;
    if (!fric_enabled_)
    {
        friction_state_ = FRCTION_RELAX;
        cnt = 0;
        return;
    }

    switch (friction_state_)
    {
    case FRCTION_RELAX:
        if (abs(feedback_.left_fric_omega) > 690.0f && abs(feedback_.right_fric_omega) > 690.0f)
        {
            friction_state_ = FRCTION_IDLE;
            cnt = 0;
        }
        break;
    case FRCTION_IDLE:
        if (abs(feedback_.left_fric_current) > 700.0f && abs(feedback_.right_fric_current) > 700.0f)
        {
            friction_state_ = FRCTION_SUSPECT;
            cnt = 0;
        }
        break;
    case FRCTION_SUSPECT:
        if (cnt >= 45)
        {
            friction_state_ = FRCTION_CONFIRMED;
            cnt = 0;
            templost = 0;
        }
        if (abs(feedback_.left_fric_current) < 500.0f || abs(feedback_.right_fric_current) < 500.0f)
        {
            if (templost++ >= 5)
            {
                friction_state_ = FRCTION_IDLE;
                cnt = 0;
                templost = 0;
            }
        }
        else
        {
            templost = 0;
        }
        break;
    case FRCTION_CONFIRMED:
        current_heat_ += 10.0f;
        friction_state_ = FRCTION_IDLE;
        cnt = 0;
        break;
    default:
        friction_state_ = FRCTION_RELAX;
        cnt = 0;
        break;
    }
}

void Shoot::update_heat_state()
{
    static float last_heat = input_.current_ref_heat_;

    // 1ms冷却
    current_heat_ -= input_.heat_cooling_rate_ * 0.001f;
    if (current_heat_ < 0.0f)
    {
        current_heat_ = 0.0f;
    }

    // 热量校准
    if (input_.current_ref_heat_ != last_heat)
    {
        current_heat_ = input_.current_ref_heat_;
        last_heat = input_.current_ref_heat_;
    }

    // 火控
    
}

void Shoot::update_trigger_state()
{
    if (!fric_enabled_)
    {
        trigger_state_ = TRIGGER_RELAX;
        return;
    }

    if (block_state_ == BLOCK_CONFIRMED || block_state_ == BLOCK_PROCESSING)
    {
        trigger_state_ = TRIGGER_BLOCK;
        return;
    }

    switch (trigger_state_)
    {
    case TRIGGER_RELAX:
        trigger_state_ = TRIGGER_IDLE;

        break;
    case TRIGGER_IDLE:

        if (shoot_mode_ == SHOOT_SINGLE || shoot_mode_ == SHOOT_DOUBLE ||
            shoot_mode_ == SHOOT_TRIPLE)
        {
            trigger_state_ = TRIGGER_ANGLE;
        }
        else if (shoot_mode_ == SHOOT_CONTINUE)
        {
            trigger_state_ = TRIGGER_SPEED;
        }
        break;
    case TRIGGER_ANGLE:
    case TRIGGER_SPEED:
        if (shoot_mode_ == SHOOT_IDLE)
        {
            trigger_state_ = TRIGGER_IDLE;
        }
        break;
    case TRIGGER_BLOCK:
        if (block_state_ == BLOCK_NORMAL || block_state_ == BLOCK_SUSPECT)
        {
            trigger_state_ = TRIGGER_IDLE;
        }
        break;
    default:
        break;
    }
}

void Shoot::update_block_state()
{
    static uint16_t cnt = 0;
    cnt++;
    switch (block_state_)
    {
    case BLOCK_NORMAL:
        if (abs(feedback_.trigger_current) > block_current_threshold_)
        {
            block_state_ = BLOCK_SUSPECT;
            cnt = 0;
        }

        break;
    case BLOCK_SUSPECT:
        if (abs(feedback_.trigger_current) < block_current_threshold_)
        {
            block_state_ = BLOCK_NORMAL;
        }
        else if (cnt > block_time_threshold_)
        {
            block_state_ = BLOCK_CONFIRMED;
            cnt = 0;
        }

        break;
    case BLOCK_CONFIRMED:
        block_state_ = BLOCK_PROCESSING;
        cnt = 0;

        break;
    case BLOCK_PROCESSING:
        if (cnt > block_recovery_time_threshold_)
        {
            block_state_ = BLOCK_NORMAL;
        }

        break;
    }
}
/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
