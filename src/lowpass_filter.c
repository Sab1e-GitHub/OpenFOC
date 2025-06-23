
#include "lowpass_filter.h"

void lowpass_filter_init(LowPassFilter_TypeDef *filter, float time_constant_sec) {
    filter->time_constant = time_constant_sec;
    filter->prev_output = 0.0f;
    filter->prev_timestamp = MICROS;
}

float lowpass_filter_update(LowPassFilter_TypeDef *filter, float input) {
    uint32_t now_us = MICROS;
    float dt = (float)((now_us - filter->prev_timestamp)) * 1e-6f;
    filter->prev_timestamp = now_us;

    if (dt < 0.0f) {
        dt = 1e-3f;
    } else if (dt > 0.3f) {
        filter->prev_output = input;
        return input;
    }

    float alpha = filter->time_constant / (filter->time_constant + dt);
    float output = alpha * filter->prev_output + (1.0f - alpha) * input;
    filter->prev_output = output;

    return output;
}
