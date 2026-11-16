/**
 * @file INS_task.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-07-13 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "bmi088.h"
#include "pid.h"
#include "quaternion_ekf.h"
#include "ins_event.h"
#include "message_center.h"
#include "message_def.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    // imu
    Bmi088_t bmi088;

    // imu温控pid
    Pid_t imu_temp_pid;
    // 加速度kf
    GravityKf_t gravity_kf;
    // 四元数ekf
    QuaternionEkf_t quaternion_ekf;

    Publisher_t publisher;
} Ins_t;

/* Exported variables ---------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern Ins_t ins;

/* Exported function declarations ---------------------------------------------*/
void INS_task(void *argument);
void INS_Init(void);
void INS_Update(void);
void INS_Publish(void);
void INS_TempControl(void);

#ifdef __cplusplus
}
#endif

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
