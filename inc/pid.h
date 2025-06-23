#ifndef OPENFOC_PID_H
#define OPENFOC_PID_H

#include "foc.h"

typedef struct {
    float kp;
    float ki;
    float kd;

    float output_min;
    float output_max;

    float integral;
    float prev_error;

    uint64_t last_timestamp_us;
} PID_Controller_TypeDef;

/**
 * @brief 初始化 PID 控制器
 */
void pid_init(PID_Controller_TypeDef *pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * @brief 更新 PID 控制器
 * @param target 目标值
 * @param measured 测量值
 * @return 控制器输出
 */
float pid_update(PID_Controller_TypeDef *pid, float target, float measured);

#endif