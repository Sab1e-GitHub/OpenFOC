#ifndef OPENFOC_FOC_H
#define OPENFOC_FOC_H

#define MICROS 1

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "lowpass_filter.h"
#include "pid.h"
#include "foc_utils.h"

#ifndef MICROS
#error "You must define the macro 'MICROS' before including this file. \
It must evaluate to a 64-bit unsigned integer (uint64_t) representing the current time in microseconds. \
Example: #define MICROS (micros64()) where micros64() returns uint64_t."
#endif

/* ---------- 常量 ---------- */
#define _PI 3.14159265359f
#define _2PI (2.0f * _PI)
#define _SQRT3 1.73205080757f
#define _PI_3 (_PI / 3.0f)

/* ---------- 枚举 ---------- */
typedef enum {
    FOC_PHASE_A = 0,
    FOC_PHASE_B = 1,
    FOC_PHASE_C = 2,
    FOC_PHASE_UNKNOWN = 255
} Motor_Phase;

typedef enum {
    FOC_OK = 0,
    FOC_ERR = -1
} FOC_Status;

typedef enum {
    FOC_CW = -1,
    FOC_CCW = 1
} FOC_Direction;

typedef enum {
    FOC_Mode_Current_Control,
    FOC_Mode_Velocity_Closedloop,
    FOC_Mode_Angle_Closedloop,
    FOC_Mode_Velocity_Openloop,
    FOC_Mode_Angle_Openloop,
    FOC_MODE_NULL=0
} FOC_Mode;

typedef enum {
    READ_CURRENTS_3_PHASE,
    READ_CURRENTS_2_PHASE
} FOC_ReadCurrentsMode;

/* ---------- 类型定义 ---------- */
typedef void (*FOC_SetPWMPtr)(float duty_ch1, float duty_ch2, float duty_ch3);
typedef void (*FOC_ReadPhaseCurrents3Ptr)(float *i1, float *i2, float *i3);
typedef void (*FOC_ReadPhaseCurrents2Ptr)(float *i1, float *i2);
typedef float (*FOC_ReadAnglePtr)(void);

/* ---------- PID 控制器 ---------- */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_limit_min;
    float output_limit_max;
} PID_Controller_TypeDef;

/* ---------- 低通滤波器 ---------- */
typedef struct {
    float prev_output;
    float alpha;
} LowPassFilter_TypeDef;

/* ---------- 电流读取函数联合体 ---------- */
typedef union {
    FOC_ReadPhaseCurrents3Ptr read_3phase;
    FOC_ReadPhaseCurrents2Ptr read_2phase;
} FOC_ReadPhaseCurrentsUnion;

/* ---------- 相位映射 ---------- */
typedef struct {
    Motor_Phase current_phase_map[3];
    Motor_Phase pwm_phase_map[3];
} FOC_PhaseCorrectionMap;

/* ---------- 模式参数结构体 ---------- */
typedef struct {
    float target_id;
    float target_iq;
} FOC_CurrentControlParams;

typedef struct {
    float target_speed_rad_per_sec;
    PID_Controller_TypeDef speed_pid;
} FOC_VelocityClosedloopParams;

typedef struct {
    float target_position_deg;
    float angle_closedloop_kv;
    PID_Controller_TypeDef position_pid;
    PID_Controller_TypeDef speed_pid;
} FOC_AngleClosedloopParams;

typedef struct {
    float target_speed_rad_per_sec;
    float uq_openloop;
} FOC_VelocityOpenloopParams;

typedef struct {
    float target_position_deg;
    float uq_openloop;
} FOC_AngleOpenloopParams;

typedef union {
    FOC_CurrentControlParams     current;
    FOC_VelocityClosedloopParams vel_closed;
    FOC_AngleClosedloopParams    angle_closed;
    FOC_VelocityOpenloopParams   vel_open;
    FOC_AngleOpenloopParams      angle_open;
} FOC_ModeParameters;

/* ---------- 回调函数 ---------- */
typedef struct {
    FOC_SetPWMPtr set_pwm;
    FOC_ReadCurrentsMode read_mode;
    FOC_ReadPhaseCurrentsUnion read_currents;
    FOC_ReadAnglePtr read_angle;
} FOC_Callbacks;

/* ---------- FOC 当前状态 ---------- */
typedef struct {
    float mech_angle_rad;
    float electric_angle_rad;
    float duty_a, duty_b, duty_c;
    float u_alpha, u_beta;
    float ia, ib, ic;
    float i_alpha, i_beta;
    float iq, id;
    float meas_iq, meas_id;
    float ud, uq;
    float speed_rad_per_sec;
    float last_theta_rad;
    uint64_t last_calc_time_us;
    bool speed_initialized;
    float last_unwrapped_angle_rad;
    float last_raw_angle_rad;
    float accumulated_angle_rad;
    uint64_t last_openloop_time_us;
} FOC_State_TypeDef;

/* ---------- 主配置结构 ---------- */
typedef struct {
    FOC_Callbacks callbacks;
    FOC_Mode mode;
    FOC_ModeParameters mode_params;
    FOC_Direction angle_sensor_direction;
    FOC_PhaseCorrectionMap phase_map;

    PID_Controller_TypeDef speed_pid_controller;
    PID_Controller_TypeDef iq_pid_controller;
    PID_Controller_TypeDef id_pid_controller;
    PID_Controller_TypeDef position_pid_controller;

    LowPassFilter_TypeDef lpf_iq;
    LowPassFilter_TypeDef lpf_id;
    LowPassFilter_TypeDef lpf_speed;

    float pole_pairs;
    float pwm_period;
    float v_dc;
    float zero_electric_angle;

    float align_voltage;
    float align_time_ms;
    float alignment_angle;
} FOC_Config_TypeDef;

/* ---------- 实例结构 ---------- */
typedef struct {
    FOC_Config_TypeDef cfg;
    FOC_State_TypeDef state;
} FOC_Instance;

/* ---------- 初始化配置封装结构 ---------- */
typedef struct {
	FOC_Mode mode;
	FOC_ModeParameters mode_params;

	float pole_pairs;
	float pwm_period;
	float v_dc;

	FOC_Direction angle_sensor_direction;
	float zero_electric_angle; // = 0 表示自动校准

	Motor_Phase phase_ch1;
	Motor_Phase phase_ch2;
	Motor_Phase phase_ch3;

	// 回调函数
	FOC_SetPWMPtr set_pwm;
	FOC_ReadCurrentsMode read_mode;
	FOC_ReadPhaseCurrents3Ptr read_currents_3;
	FOC_ReadPhaseCurrents2Ptr read_currents_2;
	FOC_ReadAnglePtr read_angle;

	// 可选参数
	float align_voltage;
	float align_time_ms;
	float alignment_angle;
} FOC_InitConfig;


// 角度转弧度
static inline float deg_to_rad(float deg) {
    return deg * (_PI / 180.0f);
}

// 弧度转角度
static inline float rad_to_deg(float rad) {
    return rad * (180.0f / _PI);
}

// 归一化角度到 [0, 2π)
static inline float normalize_angle_0_to_2pi(float angle_rad) {
    angle_rad = fmodf(angle_rad, _2PI);
    if (angle_rad < 0.0f)
        angle_rad += _2PI;
    return angle_rad;
}

// 归一化角度到 [-π, π)
static inline float normalize_angle_rad(float angle_rad) {
    angle_rad = fmodf(angle_rad + _PI, _2PI);
    if (angle_rad < 0.0f)
        angle_rad += _2PI;
    return angle_rad - _PI;
}

static inline void speed_calculate(FOC_Instance *foc) {
    uint32_t now_us = MICROS;
    FOC_State_TypeDef *state = &foc->state;

    float theta_now_rad = state->mech_angle_rad;

    if (!state->speed_initialized) {
        state->last_theta_rad = theta_now_rad;
        state->last_calc_time_us = now_us;
        state->speed_initialized = true;
        state->speed_rad_per_sec = 0.0f;
        return;
    }

    float Ts = (now_us - state->last_calc_time_us) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    float delta_theta = theta_now_rad - state->last_theta_rad;
    if (delta_theta > _PI) delta_theta -= _2PI;
    else if (delta_theta < -_PI) delta_theta += _2PI;

    state->last_theta_rad = theta_now_rad;
    state->last_calc_time_us = now_us;
    state->speed_rad_per_sec = delta_theta / Ts;
}


/* ---------- 公共函数接口 ---------- */
void foc_init(FOC_Instance *foc, FOC_InitConfig *cfg);
void foc_run(FOC_Instance *foc);
FOC_Status foc_auto_detect_phase_order(FOC_Instance *foc);
void foc_set_phase(FOC_Instance *foc, Motor_Phase ch1, Motor_Phase ch2, Motor_Phase ch3);
void foc_calibrate_zero_electric_angle(FOC_Instance *foc);



#endif