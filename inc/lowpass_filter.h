#ifndef OPENFOC_LOWPASS_FILTER_H
#define OPENFOC_LOWPASS_FILTER_H

#include "foc.h"

typedef struct {
    float time_constant;       // 时间常数 τ，单位：秒
    float prev_output;         // 上一次滤波后的输出值
    uint64_t prev_timestamp;   // 上次更新时间戳（单位：微秒）
} LowPassFilter_TypeDef;

#endif