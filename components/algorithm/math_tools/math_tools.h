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
#include "arm_math.h" // IWYU pragma: export
#include <algorithm> // IWYU pragma: export
#include <cstdlib> // IWYU pragma: export

/* Exported macros -----------------------------------------------------------*/

// rpm换算到rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)
// deg换算到rad
#define DEG_TO_RAD (PI / 180.0f)

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

float inv_sqrt(float x);

float uint_to_float(uint32_t x_int, float x_min, float x_max, size_t bits);

uint32_t float_to_uint(float x, float x_min, float x_max, size_t bits);

float cubic_map(float x, float expo);

/**
 * @brief 归化函数
 *
 * @tparam T 类型
 * @param x 传入数据
 * @param modulus 模数
 * @return T 返回的归化数, 介于 ±modulus / 2 之间
 */
template <typename T> T wrap_center(T x, T modulus)
{
    T tmp;
    tmp = std::fmod(x, modulus);
    if (tmp < -(modulus / T{2}))
    {
        tmp += modulus;
    }
    else if (tmp >= (modulus / T{2}))
    {
        tmp -= modulus;
    }
    return (tmp);
}
/************************ COPYRIGHT(C) SZTU-HJ **************************/
