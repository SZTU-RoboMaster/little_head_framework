/**
 * @file low_pass_filter.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-09-02 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "low_pass_filter.h"

#include "math_tools.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化低通滤波器截止频率
 *
 * @param
 * @param cutoff_freq
 */
void LowPassFilter::init(float cutoff_freq)
{
    fc_ = cutoff_freq;
}

float LowPassFilter::update(float input, float dt)
{
    float tau = 1.0f / (2.0f * PI * fc_);
    if (first_in_)
    {
        alpha_ = 1.0f - std::exp(-dt / tau);
        output_ = input;
        first_in_ = 0;
        return output_;
    }
    alpha_ = 1.0f - std::exp(-dt / tau);
    output_ = (1.0f - alpha_) * output_ + alpha_ * input;
    return output_;
}

void LowPassFilter::reset()
{
    first_in_ = 1;
    alpha_ = 0.0f;
    output_ = 0.0f;
}

void OneEuroFilter::init(float min_cutoff, float beta, float d_cutoff)
{
    x_filter_.init(min_cutoff);
    dx_filter_.init(d_cutoff);
    min_cutoff_ = min_cutoff;
    beta_ = beta;
}

float OneEuroFilter::update(float input, float dt)
{
    if (first_in_)
    {
        last_input_ = input;
        first_in_ = 0;
    }
    float dx = (input - last_input_) / dt;
    last_input_ = input;
    float edx = dx_filter_.update(dx, dt);
    float cutoff = min_cutoff_ + beta_ * std::abs(edx);
    x_filter_.init(cutoff);
    return x_filter_.update(input, dt);
}

void OneEuroFilter::reset()
{
    first_in_ = 1;
    last_input_ = 0.0f;
    x_filter_.reset();
    dx_filter_.reset();
}
/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
