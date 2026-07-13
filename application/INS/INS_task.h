/**
 * @file template.hpp
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
#include "bmi088.h"
#include "math_tools.h"
#include "quaternion_ekf.h"

#include "cmsis_os.h"

/* Exported macros -----------------------------------------------------------*/
#define INS_DATA_READY_FLAG (1U << 0)

extern osThreadId_t insTaskHandle;

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

    // 加速度kf
    GravityKf gravity_kf_;
    // 四元数ekf
    QuaternionEkf quaternion_ekf_;

    void init();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

/* Exported variables ---------------------------------------------------------*/
extern INS ins;

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
