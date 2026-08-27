/**
 * @file bsp_tim.cpp
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

TimManageObject tim1_manage_obj;
TimManageObject tim2_manage_obj;
TimManageObject tim3_manage_obj;
TimManageObject tim4_manage_obj;
TimManageObject tim5_manage_obj;
TimManageObject tim6_manage_obj;
TimManageObject tim7_manage_obj;
TimManageObject tim8_manage_obj;
TimManageObject tim9_manage_obj;
TimManageObject tim10_manage_obj;
TimManageObject tim11_manage_obj;
TimManageObject tim12_manage_obj;
TimManageObject tim13_manage_obj;
TimManageObject tim14_manage_obj;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化TIM定时器
 *
 * @param htim 定时器编号
 * @param callback_func 处理回调函数
 */
void tim_init(TIM_HandleTypeDef *htim, tim_callback_t callback_func)
{
    if (htim->Instance == TIM1)
    {
        tim1_manage_obj.tim_handle = htim;
        tim1_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM2)
    {
        tim2_manage_obj.tim_handle = htim;
        tim2_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM3)
    {
        tim3_manage_obj.tim_handle = htim;
        tim3_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM4)
    {
        tim4_manage_obj.tim_handle = htim;
        tim4_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM5)
    {
        tim5_manage_obj.tim_handle = htim;
        tim5_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM6)
    {
        tim6_manage_obj.tim_handle = htim;
        tim6_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM7)
    {
        tim7_manage_obj.tim_handle = htim;
        tim7_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM8)
    {
        tim8_manage_obj.tim_handle = htim;
        tim8_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM9)
    {
        tim9_manage_obj.tim_handle = htim;
        tim9_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM10)
    {
        tim10_manage_obj.tim_handle = htim;
        tim10_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM11)
    {
        tim11_manage_obj.tim_handle = htim;
        tim11_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM12)
    {
        tim12_manage_obj.tim_handle = htim;
        tim12_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM13)
    {
        tim13_manage_obj.tim_handle = htim;
        tim13_manage_obj.callback_func = callback_func;
        HAL_TIM_Base_Start_IT(htim);
    }
    else if (htim->Instance == TIM14)
    {
        tim14_manage_obj.tim_handle = htim;
        tim14_manage_obj.callback_func = callback_func;
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
        if (tim1_manage_obj.callback_func != nullptr)
        {
            tim1_manage_obj.callback_func();
        }
    }
    else if (htim->Instance == TIM2)
    {
        if (tim2_manage_obj.callback_func != nullptr)
        {
            tim2_manage_obj.callback_func();
        }
    }
    else if (htim->Instance == TIM3)
    {
        if (tim3_manage_obj.callback_func != nullptr)
        {
            tim3_manage_obj.callback_func();
        }
    }
    else if (htim->Instance == TIM4)
    {
        if (tim4_manage_obj.callback_func != nullptr)
        {
            tim4_manage_obj.callback_func();
        }
    }
    else if (htim->Instance == TIM5)
    {
        if (tim5_manage_obj.callback_func != nullptr)
        {
            tim5_manage_obj.callback_func();
        }
    }
    else if (htim->Instance == TIM7)
    {
        if (tim7_manage_obj.callback_func != nullptr)
        {
            tim7_manage_obj.callback_func();
        }
    }
    else if (htim->Instance == TIM8)
    {
        if (tim8_manage_obj.callback_func != nullptr)
        {
            tim8_manage_obj.callback_func();
        }
    }
    else if (htim->Instance == TIM9)
    {
        if (tim9_manage_obj.callback_func != nullptr)
        {
            tim9_manage_obj.callback_func();
        }
    }
    else if (htim->Instance == TIM10)
    {
        if (tim10_manage_obj.callback_func != nullptr)
        {
            tim10_manage_obj.callback_func();
        }
    }
    else if (htim->Instance == TIM11)
    {
        if (tim11_manage_obj.callback_func != nullptr)
        {
            tim11_manage_obj.callback_func();
        }
    }
    else if (htim->Instance == TIM12)
    {
        if (tim12_manage_obj.callback_func != nullptr)
        {
            tim12_manage_obj.callback_func();
        }
    }
    else if (htim->Instance == TIM13)
    {
        if (tim13_manage_obj.callback_func != nullptr)
        {
            tim13_manage_obj.callback_func();
        }
    }
    else if (htim->Instance == TIM14)
    {
        if (tim14_manage_obj.callback_func != nullptr)
        {
            tim14_manage_obj.callback_func();
        }
    }
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
