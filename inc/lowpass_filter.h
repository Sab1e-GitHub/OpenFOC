/*
 * lowpass_filter.h
 *
 *  Created on: Jun 23, 2025
 *      Author: Sab1e
 */

#ifndef OPENFOC_LOWPASS_FILTER_H_
#define OPENFOC_LOWPASS_FILTER_H_

#include <stdint.h>
#include <stddef.h>

typedef uint64_t (*GetMicrosFunc)(void); // 时间戳函数指针类型

typedef struct
{
    float time_constant;      // 时间常数 τ，单位：秒
    float prev_output;        // 上一次滤波后的输出值
    uint64_t prev_timestamp;  // 上次更新时间戳（单位：微秒）
    GetMicrosFunc get_micros; // 获取当前时间戳的函数
} LowPassFilter_TypeDef;

/**
 * @brief 初始化低通滤波器
 * @param filter            滤波器实例
 * @param time_constant_sec 时间常数（秒）
 * @param get_micros_func   获取当前微秒时间戳的函数指针（返回 uint64_t）
 */
void lowpass_filter_init(LowPassFilter_TypeDef *filter, float time_constant_sec, GetMicrosFunc get_micros_func);

/**
 * @brief 更新滤波器输入
 * @param filter 滤波器实例
 * @param input  当前输入值
 * @return 滤波后的输出值
 */
float lowpass_filter_update(LowPassFilter_TypeDef *filter, float input);

#endif /* OPENFOC_LOWPASS_FILTER_H_ */