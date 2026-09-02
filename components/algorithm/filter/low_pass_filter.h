/**
 * @file low_pass_filter.h
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-09-02 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Specialized
 *
 */
class LowPassFilter
{
public:
    LowPassFilter() = default;

    void init(float cutoff_freq);

    float update(float input, float dt);

    void reset();

protected:
    // 初始化相关常量
    float fc_ = 0.0f;

    // 常量

    // 内部变量
    uint8_t first_in_ = 1;
    float alpha_ = 0.0f;
    float output_ = 0.0f;

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

class OneEuroFilter
{
public:
    void init(float min_cutoff, float beta, float d_cutoff);

    float update(float input, float dt);

    void reset();

protected:
    // 初始化相关常量
    float fc_ = 0.0f;

    // 常量
    float beta_ = 0.0f;
    float min_cutoff_ = 0.0f;

    // 内部变量
    LowPassFilter x_filter_;
    LowPassFilter dx_filter_;

    uint8_t first_in_ = 1;
    float last_input_ = 0.0f;

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};
/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/

/*************************** COPYRIGHT(C) SZTU-HJ ******************************/
