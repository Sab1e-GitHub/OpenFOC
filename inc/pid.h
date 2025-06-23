/*
 * pid.h
 *
 *  Created on: Jun 23, 2025
 *      Author: Sab1e
 */

#ifndef OPENFOC_PID_H_
#define OPENFOC_PID_H_

#include <stdint.h>
#include <stddef.h>

typedef uint64_t (*GetMicrosFunc)(void); // 时间戳函数指针类型

typedef struct
{
    float kp;
    float ki;
    float kd;

    float output_min;
    float output_max;

    float integral;
    float prev_error;

    uint64_t last_timestamp_us;

    GetMicrosFunc get_micros; // 获取当前时间戳的函数
} PID_Controller_TypeDef;
/**
 * @brief 初始化 PID 控制器
 */
void pid_init(PID_Controller_TypeDef *pid, float kp, float ki, float kd, float out_min, float out_max, GetMicrosFunc get_micros_func);

/**
 * @brief 更新 PID 控制器
 * @param target 目标值
 * @param measured 测量值
 * @return 控制器输出
 */
float pid_update(PID_Controller_TypeDef *pid, float target, float measured);

#endif
