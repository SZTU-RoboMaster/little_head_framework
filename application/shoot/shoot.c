/**
 * @file shoot.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-07-11 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

/* Includes ------------------------------------------------------------------*/
#include "shoot.h"
#include "math_tools.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
Shoot_t shoot;

/* Private function declarations ---------------------------------------------*/
static void Shoot_Update_Friction_State(void);
static void Shoot_Update_Heat_State(void);
static void Shoot_Update_Trigger_State(void);
static void Shoot_Update_Block_State(void);

void Shoot_Init(void)
{
    shoot.config.block_current_threshold = 9000.0f;
    shoot.config.block_time_threshold = 500;
    shoot.config.block_recovery_time_threshold = 500;
    shoot.config.heat_ceasefire_threshold = 20.0f;
    shoot.config.heat_slowdown_threshold = 50.0f;
    shoot.config.fric_target_omega = 700.0f;
    shoot.vision_msg.target_lock = 50; // 49: lock, 50: unlock
    shoot.heat_first_in = 1;

    // 拨弹盘电机初始化
    Pid_Init(&shoot.trigger.angle_pid, 15.0f, 0.0f, 0.0f, 0.0f,
             0.0f, 48.0f, 0.005f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_FIRST_DISABLE);
    Pid_Init(&shoot.trigger.omega_pid, 1200.0f, 60000.0f, 0.0f, 0.0f,
             5000.0f, 10000.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_FIRST_DISABLE);
    MotorDji_Init(&shoot.trigger, &hcan2, 0x200, 0x201,
                  MOTOR_DJI_CONTROL_METHOD_ANGLE, 36.0f, 0);

    // 摩擦轮电机初始化
    Pid_Init(&shoot.friction_left.omega_pid, 40.0f, 0.0f, 0.0f, 0.0f,
             0.0f, 16384.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_FIRST_DISABLE);
    MotorDji_Init(&shoot.friction_left, &hcan2, 0x200, 0x202,
                  MOTOR_DJI_CONTROL_METHOD_OMEGA, 1.0f, 0);
    Pid_Init(&shoot.friction_right.omega_pid, 40.0f, 0.0f, 0.0f, 0.0f,
             0.0f, 16384.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_FIRST_DISABLE);
    MotorDji_Init(&shoot.friction_right, &hcan2, 0x200, 0x203,
                  MOTOR_DJI_CONTROL_METHOD_OMEGA, 1.0f, 0);

    // Topic 已由 robot_sdk 注册
    Create_Subscriber(&shoot.cmd_subscriber, kCmdShootTopicName, sizeof(shoot.cmd_msg));
    Create_Subscriber(&shoot.referee_subscriber, kRefereeTopicName, sizeof(shoot.referee_msg));
    Create_Subscriber(&shoot.vision_subscriber, kVisionTopicName, sizeof(shoot.vision_msg));
    shoot.status.mode = SHOOT_RELAX;
}

void Shoot_Update_Input(void)
{
    uint32_t cmd_version = shoot.cmd_subscriber.last_read_update_count;
    Shoot_Cmd_Message_t incoming_cmd;
    bool cmd_updated = Message_Read(&shoot.cmd_subscriber, &incoming_cmd, sizeof(incoming_cmd)) &&
                       shoot.cmd_subscriber.last_read_update_count != cmd_version;
    if (cmd_updated)
        shoot.cmd_msg = incoming_cmd;
    shoot.fric_enabled = shoot.cmd_msg.fric_enabled;
    shoot.single_shot_request =
        shoot.cmd_msg.single_shot_seq > shoot.last_single_shot_seq;
    if (cmd_updated)
        shoot.last_single_shot_seq = shoot.cmd_msg.single_shot_seq;

    Message_Read(&shoot.referee_subscriber, &shoot.referee_msg, sizeof(shoot.referee_msg));
    uint32_t vision_version = shoot.vision_subscriber.last_read_update_count;
    Vision_Message_t incoming_vision;
    if (Message_Read(&shoot.vision_subscriber, &incoming_vision, sizeof(incoming_vision)) &&
        shoot.vision_subscriber.last_read_update_count != vision_version)
        shoot.vision_msg = incoming_vision;
    shoot.auto_aim = !Message_IsTimeout(&shoot.vision_subscriber, 100) &&
                     shoot.vision_msg.target_lock == 49 && shoot.cmd_msg.auto_aim;
}

void Shoot_Update_Feedback(void)
{
    shoot.feedback.trigger_angle = shoot.trigger.rx_data.total_angle;
    shoot.feedback.trigger_omega = shoot.trigger.rx_data.omega;
    shoot.feedback.trigger_current = shoot.trigger.rx_data.current;
    shoot.feedback.left_fric_omega = shoot.friction_left.rx_data.omega;
    shoot.feedback.right_fric_omega = shoot.friction_right.rx_data.omega;
    shoot.feedback.left_fric_current = shoot.friction_left.rx_data.current;
    shoot.feedback.right_fric_current = shoot.friction_right.rx_data.current;
    shoot.feedback.current_ref_heat = shoot.referee_msg.shooter_17mm_barrel_heat;
    shoot.feedback.heat_limit = shoot.referee_msg.shooter_barrel_heat_limit;
    shoot.feedback.heat_cooling_rate = shoot.referee_msg.shooter_barrel_cooling_value;
}

void Shoot_Handle_Safety(void)
{
    // 安全处理
    if (Message_IsTimeout(&shoot.cmd_subscriber, 100))
    {
        shoot.cmd_msg = (Shoot_Cmd_Message_t){0};
        shoot.fric_enabled = 0;
        shoot.single_shot_request = 0;
    }
}

void Shoot_Set_Mode(void)
{
    if (!shoot.fric_enabled)
    {
        shoot.status.mode = SHOOT_RELAX;
        return;
    }

    if (shoot.auto_aim)
    {
        switch (shoot.status.mode)
        {
        case SHOOT_RELAX:
            shoot.status.mode = SHOOT_IDLE;
            break;
        case SHOOT_IDLE:
            if (shoot.vision_msg.fire_command == 4)
            {
                shoot.status.mode = SHOOT_CONTINUE;
                return;
            }
            if (shoot.vision_msg.fire_command == 1)
            {
                shoot.status.mode = SHOOT_SINGLE;
                shoot.single_shot_pending = 1;
                shoot.vision_msg.fire_command = 0;
            }
            break;
        case SHOOT_SINGLE:
        case SHOOT_DOUBLE:
        case SHOOT_TRIPLE:
            if (shoot.status.trigger_state == TRIGGER_BLOCK)
            {
                shoot.status.mode = SHOOT_IDLE;
                shoot.control_output.target_trigger_angle = shoot.feedback.trigger_angle;
                return;
            }
            if (fabsf(shoot.feedback.trigger_angle -
                      shoot.control_output.target_trigger_angle) < 0.002f)
                shoot.status.mode = SHOOT_IDLE;
            break;
        case SHOOT_CONTINUE:
            if (shoot.vision_msg.fire_command != 4)
                shoot.status.mode = SHOOT_IDLE;
            break;
        }
    }
    else
    {
        switch (shoot.status.mode)
        {
        case SHOOT_RELAX:
            shoot.status.mode = SHOOT_IDLE;
            break;
        case SHOOT_IDLE:
            if (shoot.cmd_msg.continue_shoot)
            {
                shoot.status.mode = SHOOT_CONTINUE;
                return;
            }
            if (shoot.single_shot_request)
            {
                shoot.status.mode = SHOOT_SINGLE;
                shoot.single_shot_pending = 1;
            }
            break;
        case SHOOT_SINGLE:
        case SHOOT_DOUBLE:
        case SHOOT_TRIPLE:
            if (shoot.status.trigger_state == TRIGGER_BLOCK)
            {
                shoot.status.mode = SHOOT_IDLE;
                shoot.control_output.target_trigger_angle = shoot.feedback.trigger_angle;
                return;
            }
            if (fabsf(shoot.feedback.trigger_angle -
                      shoot.control_output.target_trigger_angle) < 0.002f)
                shoot.status.mode = SHOOT_IDLE;
            break;
        case SHOOT_CONTINUE:
            if (!shoot.cmd_msg.continue_shoot)
                shoot.status.mode = SHOOT_IDLE;
            break;
        }
    }
}

void Shoot_Update_Control_State(void)
{
    Shoot_Update_Friction_State();
    Shoot_Update_Heat_State();
    Shoot_Update_Block_State();
    Shoot_Update_Trigger_State();
}

void Shoot_Control(void)
{
    // 摩擦轮控制
    if (!shoot.fric_enabled)
    {
        shoot.control_output.target_left_fric_omega = 0.0f;
        shoot.control_output.target_right_fric_omega = 0.0f;
    }
    else
    {
        shoot.control_output.target_left_fric_omega = -shoot.config.fric_target_omega;
        shoot.control_output.target_right_fric_omega = shoot.config.fric_target_omega;
    }

    // 拨盘控制
    switch (shoot.status.trigger_state)
    {
    case TRIGGER_RELAX:
        MotorDji_SetControlMethod(&shoot.trigger, MOTOR_DJI_CONTROL_METHOD_CURRENT);
        shoot.control_output.target_trigger_current = 0.0f;
        shoot.control_output.target_trigger_angle = shoot.feedback.trigger_angle;
        break;
    case TRIGGER_IDLE:
        MotorDji_SetControlMethod(&shoot.trigger, MOTOR_DJI_CONTROL_METHOD_OMEGA);
        shoot.control_output.target_trigger_omega = 0.0f;
        shoot.control_output.target_trigger_angle = shoot.feedback.trigger_angle;
        break;
    case TRIGGER_ANGLE:
        MotorDji_SetControlMethod(&shoot.trigger, MOTOR_DJI_CONTROL_METHOD_ANGLE);
        if (shoot.single_shot_pending)
        {
            shoot.control_output.target_trigger_angle -= 2.0f * PI / 8.0f;
            shoot.single_shot_pending = 0;
        }
        break;
    case TRIGGER_SPEED:
        MotorDji_SetControlMethod(&shoot.trigger, MOTOR_DJI_CONTROL_METHOD_OMEGA);
        shoot.control_output.target_trigger_omega = -10.0f;
        break;
    case TRIGGER_BLOCK:
        MotorDji_SetControlMethod(&shoot.trigger, MOTOR_DJI_CONTROL_METHOD_OMEGA);
        shoot.control_output.target_trigger_omega = 15.0f;
        break;
    }
}

void Shoot_Output(void)
{
    MotorDji_SetTargetAngle(&shoot.trigger, shoot.control_output.target_trigger_angle);
    MotorDji_SetTargetOmega(&shoot.trigger, shoot.control_output.target_trigger_omega);
    MotorDji_SetTargetCurrent(&shoot.trigger, shoot.control_output.target_trigger_current);
    MotorDji_SetTargetOmega(&shoot.friction_left,
                            shoot.control_output.target_left_fric_omega);
    MotorDji_SetTargetOmega(&shoot.friction_right,
                            shoot.control_output.target_right_fric_omega);
}

static void Shoot_Update_Friction_State(void)
{
    // TODO: templost不是最优解，考虑滤波或均值
    shoot.friction_count++;
    if (!shoot.fric_enabled)
    {
        shoot.status.friction_state = FRICTION_RELAX;
        shoot.friction_count = 0;
        return;
    }

    switch (shoot.status.friction_state)
    {
    case FRICTION_RELAX:
        if (fabsf(shoot.feedback.left_fric_omega) > 690.0f &&
            fabsf(shoot.feedback.right_fric_omega) > 690.0f)
        {
            shoot.status.friction_state = FRICTION_IDLE;
            shoot.friction_count = 0;
        }
        break;
    case FRICTION_IDLE:
        if (fabsf(shoot.feedback.left_fric_current) > 700.0f &&
            fabsf(shoot.feedback.right_fric_current) > 700.0f)
        {
            shoot.status.friction_state = FRICTION_SUSPECT;
            shoot.friction_count = 0;
        }
        break;
    case FRICTION_SUSPECT:
        if (shoot.friction_count >= 45)
        {
            shoot.status.friction_state = FRICTION_CONFIRMED;
            shoot.friction_count = 0;
            shoot.friction_temp_lost = 0;
        }
        if (fabsf(shoot.feedback.left_fric_current) < 500.0f ||
            fabsf(shoot.feedback.right_fric_current) < 500.0f)
        {
            if (shoot.friction_temp_lost++ >= 5)
            {
                shoot.status.friction_state = FRICTION_IDLE;
                shoot.friction_count = 0;
                shoot.friction_temp_lost = 0;
            }
        }
        else
            shoot.friction_temp_lost = 0;
        break;
    case FRICTION_CONFIRMED:
        shoot.current_heat += 10.0f;
        shoot.status.friction_state = FRICTION_IDLE;
        shoot.friction_count = 0;
        break;
    default:
        shoot.status.friction_state = FRICTION_RELAX;
        shoot.friction_count = 0;
        break;
    }
}

static void Shoot_Update_Heat_State(void)
{
    if (shoot.heat_first_in)
    {
        shoot.last_ref_heat = shoot.feedback.current_ref_heat;
        shoot.heat_first_in = 0;
    }
    // 1ms冷却
    shoot.current_heat -= shoot.feedback.heat_cooling_rate * 0.001f;
    if (shoot.current_heat < 0.0f)
        shoot.current_heat = 0.0f;

    // 热量校准
    if (shoot.feedback.current_ref_heat != shoot.last_ref_heat)
    {
        shoot.current_heat = shoot.feedback.current_ref_heat;
        shoot.last_ref_heat = shoot.feedback.current_ref_heat;
    }
    // 火控
}

static void Shoot_Update_Trigger_State(void)
{
    if (!shoot.fric_enabled)
    {
        shoot.status.trigger_state = TRIGGER_RELAX;
        return;
    }
    if (shoot.status.block_state == BLOCK_CONFIRMED ||
        shoot.status.block_state == BLOCK_PROCESSING)
    {
        shoot.status.trigger_state = TRIGGER_BLOCK;
        return;
    }

    switch (shoot.status.trigger_state)
    {
    case TRIGGER_RELAX:
        shoot.status.trigger_state = TRIGGER_IDLE;
        break;
    case TRIGGER_IDLE:
        if (shoot.status.mode == SHOOT_SINGLE || shoot.status.mode == SHOOT_DOUBLE ||
            shoot.status.mode == SHOOT_TRIPLE)
            shoot.status.trigger_state = TRIGGER_ANGLE;
        else if (shoot.status.mode == SHOOT_CONTINUE)
            shoot.status.trigger_state = TRIGGER_SPEED;
        break;
    case TRIGGER_ANGLE:
    case TRIGGER_SPEED:
        if (shoot.status.mode == SHOOT_IDLE)
            shoot.status.trigger_state = TRIGGER_IDLE;
        break;
    case TRIGGER_BLOCK:
        if (shoot.status.block_state == BLOCK_NORMAL ||
            shoot.status.block_state == BLOCK_SUSPECT)
            shoot.status.trigger_state = TRIGGER_IDLE;
        break;
    default:
        break;
    }
}

static void Shoot_Update_Block_State(void)
{
    shoot.block_count++;
    switch (shoot.status.block_state)
    {
    case BLOCK_NORMAL:
        if (fabsf(shoot.feedback.trigger_current) > shoot.config.block_current_threshold)
        {
            shoot.status.block_state = BLOCK_SUSPECT;
            shoot.block_count = 0;
        }
        break;
    case BLOCK_SUSPECT:
        if (fabsf(shoot.feedback.trigger_current) < shoot.config.block_current_threshold)
            shoot.status.block_state = BLOCK_NORMAL;
        else if (shoot.block_count > shoot.config.block_time_threshold)
        {
            shoot.status.block_state = BLOCK_CONFIRMED;
            shoot.block_count = 0;
        }
        break;
    case BLOCK_CONFIRMED:
        shoot.status.block_state = BLOCK_PROCESSING;
        shoot.block_count = 0;
        break;
    case BLOCK_PROCESSING:
        if (shoot.block_count > shoot.config.block_recovery_time_threshold)
            shoot.status.block_state = BLOCK_NORMAL;
        break;
    }
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
