/**
 * @file bsp_tim.c
 * @author anchengc
 * @brief 移植USTC-RoboWalker的TIM定时器初始化与配置流程
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_tim.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

BspTim_Manage_t bsp_tim1;
BspTim_Manage_t bsp_tim2;
BspTim_Manage_t bsp_tim3;
BspTim_Manage_t bsp_tim4;
BspTim_Manage_t bsp_tim5;
BspTim_Manage_t bsp_tim6;
BspTim_Manage_t bsp_tim7;
BspTim_Manage_t bsp_tim8;
BspTim_Manage_t bsp_tim9;
BspTim_Manage_t bsp_tim10;
BspTim_Manage_t bsp_tim11;
BspTim_Manage_t bsp_tim12;
BspTim_Manage_t bsp_tim13;
BspTim_Manage_t bsp_tim14;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化TIM定时器
 *
 * @param htim 定时器编号
 * @param callback_func 处理回调函数
 */
void BspTim_Init(TIM_HandleTypeDef *htim, BspTim_Callback_t callback_func)
{
    if (htim->Instance == TIM1)
    {
        bsp_tim1.tim_handle = htim;
        bsp_tim1.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM2)
    {
        bsp_tim2.tim_handle = htim;
        bsp_tim2.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM3)
    {
        bsp_tim3.tim_handle = htim;
        bsp_tim3.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM4)
    {
        bsp_tim4.tim_handle = htim;
        bsp_tim4.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM5)
    {
        bsp_tim5.tim_handle = htim;
        bsp_tim5.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM6)
    {
        bsp_tim6.tim_handle = htim;
        bsp_tim6.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM7)
    {
        bsp_tim7.tim_handle = htim;
        bsp_tim7.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM8)
    {
        bsp_tim8.tim_handle = htim;
        bsp_tim8.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM9)
    {
        bsp_tim9.tim_handle = htim;
        bsp_tim9.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM10)
    {
        bsp_tim10.tim_handle = htim;
        bsp_tim10.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM11)
    {
        bsp_tim11.tim_handle = htim;
        bsp_tim11.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM12)
    {
        bsp_tim12.tim_handle = htim;
        bsp_tim12.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM13)
    {
        bsp_tim13.tim_handle = htim;
        bsp_tim13.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM14)
    {
        bsp_tim14.tim_handle = htim;
        bsp_tim14.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
}

/**
 * @brief HAL库TIM定时器中断
 *
 * @param htim TIM编号
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }

    // 判断程序初始化完成
    if (!initialized)
    {
        return;
    }

    // 选择回调函数
    if (htim->Instance == TIM1)
    {
        if (bsp_tim1.callback_func != NULL)
        {
            bsp_tim1.callback_func();
        }
    }
    else if (htim->Instance == TIM2)
    {
        if (bsp_tim2.callback_func != NULL)
        {
            bsp_tim2.callback_func();
        }
    }
    else if (htim->Instance == TIM3)
    {
        if (bsp_tim3.callback_func != NULL)
        {
            bsp_tim3.callback_func();
        }
    }
    else if (htim->Instance == TIM4)
    {
        if (bsp_tim4.callback_func != NULL)
        {
            bsp_tim4.callback_func();
        }
    }
    else if (htim->Instance == TIM5)
    {
        if (bsp_tim5.callback_func != NULL)
        {
            bsp_tim5.callback_func();
        }
    }
    else if (htim->Instance == TIM7)
    {
        if (bsp_tim7.callback_func != NULL)
        {
            bsp_tim7.callback_func();
        }
    }
    else if (htim->Instance == TIM8)
    {
        if (bsp_tim8.callback_func != NULL)
        {
            bsp_tim8.callback_func();
        }
    }
    else if (htim->Instance == TIM9)
    {
        if (bsp_tim9.callback_func != NULL)
        {
            bsp_tim9.callback_func();
        }
    }
    else if (htim->Instance == TIM10)
    {
        if (bsp_tim10.callback_func != NULL)
        {
            bsp_tim10.callback_func();
        }
    }
    else if (htim->Instance == TIM11)
    {
        if (bsp_tim11.callback_func != NULL)
        {
            bsp_tim11.callback_func();
        }
    }
    else if (htim->Instance == TIM12)
    {
        if (bsp_tim12.callback_func != NULL)
        {
            bsp_tim12.callback_func();
        }
    }
    else if (htim->Instance == TIM13)
    {
        if (bsp_tim13.callback_func != NULL)
        {
            bsp_tim13.callback_func();
        }
    }
    else if (htim->Instance == TIM14)
    {
        if (bsp_tim14.callback_func != NULL)
        {
            bsp_tim14.callback_func();
        }
    }
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
