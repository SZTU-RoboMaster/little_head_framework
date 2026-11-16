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

#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include <stdint.h>

typedef struct
{
    float fc;
    float alpha;
    float output;
    uint8_t first_in;
} LowPassFilter_t;

typedef struct
{
    LowPassFilter_t x_filter;
    LowPassFilter_t dx_filter;
    float min_cutoff;
    float beta;
    float last_input;
    uint8_t first_in;
} OneEuroFilter_t;

#ifdef __cplusplus
extern "C" {
#endif

void LowPassFilter_Init(LowPassFilter_t *filter, float cutoff_freq);
float LowPassFilter_Update(LowPassFilter_t *filter, float input, float dt);
void LowPassFilter_Reset(LowPassFilter_t *filter);

void OneEuroFilter_Init(OneEuroFilter_t *filter, float min_cutoff, float beta, float d_cutoff);
float OneEuroFilter_Update(OneEuroFilter_t *filter, float input, float dt);
void OneEuroFilter_Reset(OneEuroFilter_t *filter);

#ifdef __cplusplus
}
#endif

#endif
