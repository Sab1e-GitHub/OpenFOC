/*
 * foc_utils.h
 *
 *  Created on: Jun 23, 2025
 *      Author: Sab1e
 */

#ifndef OPENFOC_FOC_MATH_H_
#define OPENFOC_FOC_MATH_H_

#include <math.h>
#include <stdint.h>

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))
#define _round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x) - 0.5))
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ((a) != (NOT_SET))

#define _2_SQRT3 1.15470053838
#define _SQRT3 1.73205080757
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038
#define _PI_6 0.52359877559

typedef uint64_t (*GetMicrosFunc)(void); // 时间戳函数指针类型

float _sin(float a);
float _cos(float a);
float _sqrtApprox(float number);
void foc_delay_us(GetMicrosFunc micros, uint64_t us);
void foc_delay_ms(GetMicrosFunc micros, uint64_t ms);

// 角度转弧度
static inline float deg_to_rad(float deg)
{
    return deg * (_PI / 180.0f);
}

// 弧度转角度
static inline float rad_to_deg(float rad)
{
    return rad * (180.0f / _PI);
}

// 归一化角度到 [0, 2π)
static inline float normalize_angle_0_to_2pi(float angle_rad)
{
    angle_rad = fmodf(angle_rad, _2PI);
    if (angle_rad < 0.0f)
        angle_rad += _2PI;
    return angle_rad;
}

// 归一化角度到 [-π, π)
static inline float normalize_angle_rad(float angle_rad)
{
    angle_rad = fmodf(angle_rad + _PI, _2PI);
    if (angle_rad < 0.0f)
        angle_rad += _2PI;
    return angle_rad - _PI;
}

static inline void park_transform(float alpha, float beta, float angle_rad, float *d, float *q)
{
    float cos_theta = _cos(angle_rad);
    float sin_theta = _sin(angle_rad);
    *d = alpha * cos_theta + beta * sin_theta;
    *q = -alpha * sin_theta + beta * cos_theta;
}

static inline void inv_park_transform(float d, float q, float angle_rad, float *alpha, float *beta)
{
    float cos_theta = _cos(angle_rad);
    float sin_theta = _sin(angle_rad);
    *alpha = d * cos_theta - q * sin_theta;
    *beta = d * sin_theta + q * cos_theta;
}

static inline void clarke_transform(float Ia, float Ib, float Ic, float *alpha, float *beta)
{
    if (Ic == 0)
    {
        *alpha = Ia;
        *beta = (Ia + 2.0f * Ib) * _1_SQRT3;
    }
    else
    {
        *alpha = (2.0f * Ia - Ib - Ic) * (1.0f / 3.0f);
        *beta = (Ib - Ic) * _1_SQRT3 * (1.0f / 3.0f);
    }
}

#endif