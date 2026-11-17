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
#include "tim.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief TIM定时器回调函数数据类型
 *
 */
typedef void (*BspTim_Callback_t)(void);

/**
 * @brief TIM定时器处理结构体
 *
 */
typedef struct
{
    TIM_HandleTypeDef *tim_handle;
    BspTim_Callback_t callback_func;
} BspTim_Manage_t;

/* Exported variables ---------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t initialized;

extern BspTim_Manage_t bsp_tim1;
extern BspTim_Manage_t bsp_tim2;
extern BspTim_Manage_t bsp_tim3;
extern BspTim_Manage_t bsp_tim4;
extern BspTim_Manage_t bsp_tim5;
extern BspTim_Manage_t bsp_tim6;
extern BspTim_Manage_t bsp_tim7;
extern BspTim_Manage_t bsp_tim8;
extern BspTim_Manage_t bsp_tim15;
extern BspTim_Manage_t bsp_tim16;
extern BspTim_Manage_t bsp_tim17;
extern BspTim_Manage_t bsp_tim20;

/* Exported function declarations ---------------------------------------------*/

void BspTim_Init(TIM_HandleTypeDef *htim, BspTim_Callback_t callback_func);

#ifdef __cplusplus
}
#endif

/************************ COPYRIGHT(C) SZTU-HJ **************************/
