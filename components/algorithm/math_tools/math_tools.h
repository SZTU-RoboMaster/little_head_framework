/**
 * @file math_tools.h
 * @author anchengc
 * @brief 数学工具函数声明
 * @version 0.1
 * @date 2026-06-05 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "arm_math.h"
#include "stm32f4xx_hal.h"


/* Exported macros -----------------------------------------------------------*/

// rpm换算到rad/s
#define RPM_TO_RADPS (2.0f * M_PI / 60.0f)
// deg换算到rad
#define DEG_TO_RAD (M_PI / 180.0f)

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

float inv_sqrt(float x);

float uint_to_float(uint32_t x_int, float x_min, float x_max, size_t bits);

uint32_t float_to_uint(float x, float x_min, float x_max, size_t bits);

/**
 * @brief 限幅函数
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param min 最小值
 * @param max 最大值
 */
template <typename Type> Type math_constrain(Type *x, Type min, Type max)
{
    if (*x < min)
    {
        *x = min;
    }
    else if (*x > max)
    {
        *x = max;
    }
    return (*x);
}

/**
 * @brief 求绝对值
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @return Type x的绝对值
 */
template <typename Type> Type math_abs(Type x)
{
    return ((x > 0) ? x : -x);
}

/**
 * @brief 求取模归化
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param modulus 模数
 * @return Type 返回的归化数, 介于 ±modulus / 2 之间
 */
template <typename Type> Type math_modulus_normalize(Type x, Type modulus)
{
    float tmp;

    tmp = fmod(x + modulus / 2.0f, modulus);

    if (tmp < 0.0f)
    {
        tmp += modulus;
    }

    return (tmp - modulus / 2.0f);
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
