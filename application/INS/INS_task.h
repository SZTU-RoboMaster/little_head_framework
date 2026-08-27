/**
 * @file INS_task.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-07-13 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "bmi088.h"
#include "pid.h"
#include "quaternion_ekf.h"

#include "cmsis_os2.h"

#include "message_center.h"
#include "message_def.h"

/* Exported macros -----------------------------------------------------------*/
#define INS_DATA_READY_FLAG (1U << 0)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Specialized
 *
 */
class INS
{
public:
    // imu
    Bmi088 bmi088_;

    void init();

    void update();

    void publish();

    void temp_control();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // imu温控pid
    Pid imu_temp_pid_;
    // 加速度kf
    GravityKf gravity_kf_;
    // 四元数ekf
    QuaternionEkf quaternion_ekf_;

    // 读变量

    // 写变量
    Publisher<InsMessage> publisher_;

    // 读写变量

    // 内部函数
};

/* Exported variables ---------------------------------------------------------*/
extern INS ins;

extern osThreadId_t insTaskHandle;

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
