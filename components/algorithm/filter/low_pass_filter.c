#include "low_pass_filter.h"

#include "math_tools.h"
#include <math.h>
#include <stddef.h>

void LowPassFilter_Init(LowPassFilter_t *filter, float cutoff_freq)
{
    if (filter == NULL)
    {
        return;
    }

    filter->fc = cutoff_freq;
    LowPassFilter_Reset(filter);
}

float LowPassFilter_Update(LowPassFilter_t *filter, float input, float dt)
{
    if (filter == NULL || filter->fc <= 0.0f || dt <= 0.0f)
    {
        return input;
    }

    float tau = 1.0f / (2.0f * PI * filter->fc);
    filter->alpha = 1.0f - expf(-dt / tau);

    if (filter->first_in)
    {
        filter->output = input;
        filter->first_in = 0;
    }
    else
    {
        filter->output = (1.0f - filter->alpha) * filter->output + filter->alpha * input;
    }

    return filter->output;
}

void LowPassFilter_Reset(LowPassFilter_t *filter)
{
    if (filter == NULL)
    {
        return;
    }

    filter->first_in = 1;
    filter->alpha = 0.0f;
    filter->output = 0.0f;
}

void OneEuroFilter_Init(OneEuroFilter_t *filter, float min_cutoff, float beta, float d_cutoff)
{
    if (filter == NULL)
    {
        return;
    }

    filter->min_cutoff = min_cutoff;
    filter->beta = beta;
    LowPassFilter_Init(&filter->x_filter, min_cutoff);
    LowPassFilter_Init(&filter->dx_filter, d_cutoff);
    filter->first_in = 1;
    filter->last_input = 0.0f;
}

float OneEuroFilter_Update(OneEuroFilter_t *filter, float input, float dt)
{
    if (filter == NULL || dt <= 0.0f)
    {
        return input;
    }

    if (filter->first_in)
    {
        filter->last_input = input;
        filter->first_in = 0;
    }

    float dx = (input - filter->last_input) / dt;
    filter->last_input = input;
    float edx = LowPassFilter_Update(&filter->dx_filter, dx, dt);
    float cutoff = filter->min_cutoff + filter->beta * fabsf(edx);

    /* 调整截止频率时保留 x_filter 的历史输出。 */
    filter->x_filter.fc = cutoff;
    return LowPassFilter_Update(&filter->x_filter, input, dt);
}

void OneEuroFilter_Reset(OneEuroFilter_t *filter)
{
    if (filter == NULL)
    {
        return;
    }

    filter->first_in = 1;
    filter->last_input = 0.0f;
    LowPassFilter_Reset(&filter->x_filter);
    LowPassFilter_Reset(&filter->dx_filter);
}
