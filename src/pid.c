/*
 * pid.c
 *
 *  Created on: Jun 23, 2025
 *      Author: Sab1e
 */

#include "pid.h"

void pid_init(PID_Controller_TypeDef *pid, float kp, float ki, float kd, float out_min, float out_max, GetMicrosFunc get_micros_func)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output_min = out_min;
    pid->output_max = out_max;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->get_micros = get_micros_func;
    pid->last_timestamp_us = pid->get_micros();
}

float pid_update(PID_Controller_TypeDef *pid, float target, float measured)
{
	if(pid->get_micros == NULL)return 0.0f;
    uint64_t now_us = pid->get_micros();
    float dt = (now_us - pid->last_timestamp_us) * 1e-6f;

    // 防止异常 dt
    if (dt <= 0.0f || dt > 0.3f)
    {
        dt = 1e-3f;
    }

    pid->last_timestamp_us = now_us;

    float error = target - measured;

    // 积分项
    pid->integral += error * dt;

    // 微分项
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // 输出限幅
    if (output > pid->output_max)
        output = pid->output_max;
    else if (output < pid->output_min)
        output = pid->output_min;

    return output;
}
