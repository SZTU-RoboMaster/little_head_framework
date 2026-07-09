/**
 * @file math_tools.cpp
 * @author anchengc
 * @brief 数学工具函数实现
 * @version 0.1
 * @date 2026-06-05 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "math_tools.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 快速计算逆平方根
 *
 * @param x 待计算的浮点数
 * @note ref: https://en.wikipedia.org/wiki/Fast_inverse_square_root
 * @return float 平方根的倒数
 */
float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 将无符号整数映射到浮点数
 *
 * @param x_int 无符号整数
 * @param x_min 浮点数最小值
 * @param x_max 浮点数最大值
 * @param bits 无符号整数的位宽
 * @return float 映射后的浮点数
 */
float uint_to_float(uint32_t x_int, float x_min, float x_max, size_t bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 将浮点数映射到无符号整数
 *
 * @param x 浮点数
 * @param x_min 浮点数最小值
 * @param x_max 浮点数最大值
 * @param bits 无符号整数的位宽
 * @return uint32_t 无符号整数
 */
uint32_t float_to_uint(float x, float x_min, float x_max, size_t bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (uint32_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/************************ COPYRIGHT(C) SZTU-HJ **************************/
