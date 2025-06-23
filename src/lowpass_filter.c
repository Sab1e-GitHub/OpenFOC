/*
 * lowpass_filter.c
 *
 *  Created on: Jun 23, 2025
 *      Author: Sab1e
 */

#include "lowpass_filter.h"

void lowpass_filter_init(LowPassFilter_TypeDef *filter, float time_constant_sec, GetMicrosFunc get_micros_func)
{
    filter->time_constant = time_constant_sec;
    filter->prev_output = 0.0f;
    filter->get_micros = get_micros_func;
    filter->prev_timestamp = (get_micros_func != NULL) ? get_micros_func() : 0;
}

float lowpass_filter_update(LowPassFilter_TypeDef *filter, float input)
{
    if (filter == NULL || filter->get_micros == NULL)
        return input;

    uint64_t now_us = filter->get_micros();
    float dt = (float)(now_us - filter->prev_timestamp) * 1e-6f;
    filter->prev_timestamp = now_us;

    if (dt < 0.0f)
    {
        dt = 1e-3f; // fallback
    }
    else if (dt > 0.3f)
    {
        filter->prev_output = input;
        return input;
    }

    float alpha = filter->time_constant / (filter->time_constant + dt);
    float output = alpha * filter->prev_output + (1.0f - alpha) * input;
    filter->prev_output = output;

    return output;
}
