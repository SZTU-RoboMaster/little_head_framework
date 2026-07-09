/**
 * @file bsp_tim.h
 * @author anchengc
 * @brief 移植USTC-RoboWalker的TIM定时器初始化与配置流程
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "tim.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief TIM定时器回调函数数据类型
 *
 */
typedef void (*tim_callback_t)();

/**
 * @brief TIM定时器处理结构体
 *
 */
struct TimManageObject
{
    TIM_HandleTypeDef *tim_handle;
    tim_callback_t callback_func;
};

/* Exported variables ---------------------------------------------------------*/
extern "C" uint8_t init_finished;

extern TimManageObject tim1_manage_obj;
extern TimManageObject tim2_manage_obj;
extern TimManageObject tim3_manage_obj;
extern TimManageObject tim4_manage_obj;
extern TimManageObject tim5_manage_obj;
extern TimManageObject tim6_manage_obj;
extern TimManageObject tim7_manage_obj;
extern TimManageObject tim8_manage_obj;
extern TimManageObject tim9_manage_obj;
extern TimManageObject tim10_manage_obj;
extern TimManageObject tim11_manage_obj;
extern TimManageObject tim12_manage_obj;
extern TimManageObject tim13_manage_obj;
extern TimManageObject tim14_manage_obj;

/* Exported function declarations ---------------------------------------------*/

void tim_init(TIM_HandleTypeDef *htim, tim_callback_t callback_func);

/************************ COPYRIGHT(C) SZTU-HJ **************************/
