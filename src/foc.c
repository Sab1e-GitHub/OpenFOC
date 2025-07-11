/*
 * foc.c
 *
 *  Created on: Jun 23, 2025
 *      Author: Sab1e
 */

#include "foc.h"



void foc_set_phase(FOC_Instance *foc, Motor_Phase ch1, Motor_Phase ch2,
                   Motor_Phase ch3)
{
    foc->cfg.phase_map.current_phase_map[0] = ch1;
    foc->cfg.phase_map.pwm_phase_map[0] = ch1;
    foc->cfg.phase_map.current_phase_map[1] = ch2;
    foc->cfg.phase_map.pwm_phase_map[1] = ch2;
    foc->cfg.phase_map.current_phase_map[2] = ch3;
    foc->cfg.phase_map.pwm_phase_map[2] = ch3;
}

void foc_init(FOC_Instance *foc, const FOC_InitConfig *config)
{
    if (foc == NULL || config == NULL)
        return;

    FOC_Config_TypeDef *cfg = &foc->cfg;

    /* ------------ 模式与参数 ------------ */
    cfg->mode = config->mode;
    cfg->mode_params = config->mode_params;

    /* ------------ 电机参数 ------------ */
    cfg->pole_pairs = config->pole_pairs;
    cfg->pwm_period = config->pwm_period;
    cfg->v_dc = config->v_dc;

    /* ------------ 对齐参数 ------------ */
    cfg->align_voltage = (config->align_voltage > 0.0f) ? config->align_voltage : 2.5f;
    cfg->align_time_ms = (config->align_time_ms > 0.0f) ? config->align_time_ms : 1000.0f;
    cfg->alignment_angle = (config->alignment_angle > 0.0f) ? config->alignment_angle : (3.0f * _PI / 2.0f);

    /* ------------ 传感器方向 ------------ */
    cfg->angle_sensor_direction = config->angle_sensor_direction;

    /* ------------ 回调函数 ------------ */
    cfg->callbacks.set_pwm = config->set_pwm;
    cfg->callbacks.read_mode = config->read_mode;
    cfg->callbacks.read_angle = config->read_angle;
    if (config->read_mode == READ_CURRENTS_3_PHASE)
    {
        cfg->callbacks.read_currents.read_3phase = config->read_currents_3;
    }
    else if (config->read_mode == READ_CURRENTS_2_PHASE)
    {
        cfg->callbacks.read_currents.read_2phase = config->read_currents_2;
    }
    cfg->get_micros = config->get_micros;

    /* ------------ 相序设置 ------------ */
    foc_set_phase(foc, config->phase_ch1, config->phase_ch2, config->phase_ch3);

    /* ------------ 零电角设置 ------------ */
    if (config->zero_electric_angle != 0.0f)
    {
        cfg->zero_electric_angle = config->zero_electric_angle;
    }
    else
    {
        foc_calibrate_zero_electric_angle(foc);
    }

    /* ------------ PID 初始化 ------------ */
    switch (config->mode) {
    case FOC_Mode_Current_Control:
        pid_init(&cfg->iq_pid_controller,
                 config->mode_params.current.iq_pid.kp,
                 config->mode_params.current.iq_pid.ki,
                 config->mode_params.current.iq_pid.kd,
                 -config->mode_params.current.iq_pid.out_max,
                 config->mode_params.current.iq_pid.out_max,
                 config->get_micros);
        pid_init(&cfg->id_pid_controller,
                 config->mode_params.current.id_pid.kp,
                 config->mode_params.current.id_pid.ki,
                 config->mode_params.current.id_pid.kd,
                 -config->mode_params.current.id_pid.out_max,
                 config->mode_params.current.id_pid.out_max,
                 config->get_micros);
        break;
    case FOC_Mode_Velocity_Closedloop:
        pid_init(&cfg->speed_pid_controller,
                 config->mode_params.vel_closed.speed_pid.kp,
                 config->mode_params.vel_closed.speed_pid.ki,
                 config->mode_params.vel_closed.speed_pid.kd,
                 -config->mode_params.vel_closed.speed_pid.out_max,
                 config->mode_params.vel_closed.speed_pid.out_max,
                 config->get_micros);
        pid_init(&cfg->iq_pid_controller,
                 config->mode_params.vel_closed.iq_pid.kp,
                 config->mode_params.vel_closed.iq_pid.ki,
                 config->mode_params.vel_closed.iq_pid.kd,
                 -config->mode_params.vel_closed.iq_pid.out_max,
                 config->mode_params.vel_closed.iq_pid.out_max,
                 config->get_micros);
        pid_init(&cfg->id_pid_controller,
                 config->mode_params.vel_closed.id_pid.kp,
                 config->mode_params.vel_closed.id_pid.ki,
                 config->mode_params.vel_closed.id_pid.kd,
                 -config->mode_params.vel_closed.id_pid.out_max,
                 config->mode_params.vel_closed.id_pid.out_max,
                 config->get_micros);
        break;
    case FOC_Mode_Angle_Closedloop:
        pid_init(&cfg->position_pid_controller,
                 config->mode_params.angle_closed.position_pid.kp,
                 config->mode_params.angle_closed.position_pid.ki,
                 config->mode_params.angle_closed.position_pid.kd,
                 -config->mode_params.angle_closed.position_pid.out_max,
                 config->mode_params.angle_closed.position_pid.out_max,
                 config->get_micros);
        pid_init(&cfg->speed_pid_controller,
                 config->mode_params.angle_closed.speed_pid.kp,
                 config->mode_params.angle_closed.speed_pid.ki,
                 config->mode_params.angle_closed.speed_pid.kd,
                 -config->mode_params.angle_closed.speed_pid.out_max,
                 config->mode_params.angle_closed.speed_pid.out_max,
                 config->get_micros);
        pid_init(&cfg->iq_pid_controller,
                 config->mode_params.angle_closed.iq_pid.kp,
                 config->mode_params.angle_closed.iq_pid.ki,
                 config->mode_params.angle_closed.iq_pid.kd,
                 -config->mode_params.angle_closed.iq_pid.out_max,
                 config->mode_params.angle_closed.iq_pid.out_max,
                 config->get_micros);
        pid_init(&cfg->id_pid_controller,
                 config->mode_params.angle_closed.id_pid.kp,
                 config->mode_params.angle_closed.id_pid.ki,
                 config->mode_params.angle_closed.id_pid.kd,
                 -config->mode_params.angle_closed.id_pid.out_max,
                 config->mode_params.angle_closed.id_pid.out_max,
                 config->get_micros);
        break;
    default:
        // openloop等模式无需PID
        break;
    }

    /* ------------ 低通滤波器初始化 ------------ */
    float lpf_speed_ts = 0.01f; // 默认值，可根据需要调整

    switch (config->mode) {
    case FOC_Mode_Current_Control:
        if (config->mode_params.current.lpf_speed_ts > 0)
            lpf_speed_ts = config->mode_params.current.lpf_speed_ts;
		lowpass_filter_init(&cfg->lpf_iq,
				config->mode_params.current.lpf_iq_ts,
				config->get_micros);
		lowpass_filter_init(&cfg->lpf_id,
			config->mode_params.current.lpf_id_ts,
			config->get_micros);
        break;
    case FOC_Mode_Velocity_Closedloop:
        if (config->mode_params.vel_closed.lpf_speed_ts > 0)
            lpf_speed_ts = config->mode_params.vel_closed.lpf_speed_ts;
        lowpass_filter_init(&cfg->lpf_iq,
            config->mode_params.vel_closed.lpf_iq_ts,
            config->get_micros);
        lowpass_filter_init(&cfg->lpf_id,
            config->mode_params.vel_closed.lpf_id_ts,
            config->get_micros);
        break;
    case FOC_Mode_Angle_Closedloop:
        if (config->mode_params.angle_closed.lpf_speed_ts > 0)
            lpf_speed_ts = config->mode_params.angle_closed.lpf_speed_ts;
        lowpass_filter_init(&cfg->lpf_iq,
            config->mode_params.angle_closed.lpf_iq_ts,
            config->get_micros);
        lowpass_filter_init(&cfg->lpf_id,
            config->mode_params.angle_closed.lpf_id_ts,
            config->get_micros);
        break;
    case FOC_Mode_Velocity_Openloop:
        if (config->mode_params.vel_open.lpf_speed_ts > 0)
            lpf_speed_ts = config->mode_params.vel_open.lpf_speed_ts;
        break;
    case FOC_Mode_Angle_Openloop:
        if (config->mode_params.angle_open.lpf_speed_ts > 0)
            lpf_speed_ts = config->mode_params.angle_open.lpf_speed_ts;
        break;
    default:
        break;
    }
    lowpass_filter_init(&cfg->lpf_speed, lpf_speed_ts, config->get_micros);

    /* ------------ 状态清零 ------------ */
    memset(&foc->state, 0, sizeof(FOC_State_TypeDef));
}

void foc_set_corrected_pwm(FOC_Instance *foc)
{
    if (foc == NULL || foc->cfg.callbacks.set_pwm == NULL)
        return;

    const Motor_Phase *map = foc->cfg.phase_map.pwm_phase_map;

    // 从 state 获取标准 ABC 的 duty
    float duty[3] = {
        foc->state.duty_a,
        foc->state.duty_b,
        foc->state.duty_c};

    // 按照 PWM 通道顺序排列
    float out_duty_ch1 = duty[map[0]];
    float out_duty_ch2 = duty[map[1]];
    float out_duty_ch3 = duty[map[2]];

    // 调用底层 set_pwm 接口
    foc->cfg.callbacks.set_pwm(out_duty_ch1, out_duty_ch2, out_duty_ch3);
}

void foc_read_corrected_currents(FOC_Instance *foc)
{
    if (foc == NULL)
        return;

    const Motor_Phase *map = foc->cfg.phase_map.current_phase_map;
    float currents[3] = {0};

    if (foc->cfg.callbacks.read_mode == READ_CURRENTS_3_PHASE)
    {
        float raw_ia = 0, raw_ib = 0, raw_ic = 0;
        foc->cfg.callbacks.read_currents.read_3phase(&raw_ia, &raw_ib, &raw_ic);

        currents[map[0]] = raw_ia;
        currents[map[1]] = raw_ib;
        currents[map[2]] = raw_ic;
    }
    else if (foc->cfg.callbacks.read_mode == READ_CURRENTS_2_PHASE)
    {
        float i1 = 0, i2 = 0;
        foc->cfg.callbacks.read_currents.read_2phase(&i1, &i2);

        currents[map[0]] = i1;
        currents[map[1]] = i2;
        currents[map[2]] = -i1 - i2; // 基于KCL估算第三相
    }

    foc->state.ia = currents[FOC_PHASE_A];
    foc->state.ib = currents[FOC_PHASE_B];
    foc->state.ic = currents[FOC_PHASE_C];
}

/**
 * @brief 计算SVPWM
 * @param α-β坐标系下的Uα Uβ（单位：V）
 * @return FOC_Status
 */
static inline void svpwm_calculate(FOC_Config_TypeDef *cfg, float u_alpha,
                                   float u_beta, float *duty_a, float *duty_b, float *duty_c)
{

    // 1. 内部归一化
    float norm_factor = cfg->v_dc / _SQRT3;
    float u_alpha_norm = u_alpha / norm_factor;
    float u_beta_norm = u_beta / norm_factor;

    // 2. 限制输入在 [-1.0, 1.0]
    u_alpha_norm = fminf(fmaxf(u_alpha_norm, -1.0f), 1.0f);
    u_beta_norm = fminf(fmaxf(u_beta_norm, -1.0f), 1.0f);

    // 3. 电压矢量幅值
    float v_ref = _sqrt(
        u_alpha_norm * u_alpha_norm + u_beta_norm * u_beta_norm);
    if (v_ref > 1.0f)
    {
        float scale = 1.0f / v_ref;
        u_alpha_norm *= scale;
        u_beta_norm *= scale;
        v_ref = 1.0f;
    }

    // 4. 角度与扇区
    float angle = atan2f(u_beta_norm, u_alpha_norm);
    if (angle < 0.0f)
        angle += _2PI;

    uint8_t sector = (uint8_t)(angle / _PI_3);
    float sector_start_angle = sector * _PI_3;
    float angle_in_sector = angle - sector_start_angle;

    // 5. 计算T1、T2、T0
    float T1 = cfg->pwm_period * v_ref * _sin(_PI_3 - angle_in_sector);
    float T2 = cfg->pwm_period * v_ref * _sin(angle_in_sector);
    float T0 = cfg->pwm_period - T1 - T2;

    // 6. 计算占空比
    float Ta, Tb, Tc;

    switch (sector)
    {
    case 0:
        Ta = (T1 + T2 + T0 * 0.5f) / cfg->pwm_period;
        Tb = (T2 + T0 * 0.5f) / cfg->pwm_period;
        Tc = T0 * 0.5f / cfg->pwm_period;
        break;
    case 1:
        Ta = (T1 + T0 * 0.5f) / cfg->pwm_period;
        Tb = (T1 + T2 + T0 * 0.5f) / cfg->pwm_period;
        Tc = T0 * 0.5f / cfg->pwm_period;
        break;
    case 2:
        Ta = T0 * 0.5f / cfg->pwm_period;
        Tb = (T1 + T2 + T0 * 0.5f) / cfg->pwm_period;
        Tc = (T2 + T0 * 0.5f) / cfg->pwm_period;
        break;
    case 3:
        Ta = T0 * 0.5f / cfg->pwm_period;
        Tb = (T1 + T0 * 0.5f) / cfg->pwm_period;
        Tc = (T1 + T2 + T0 * 0.5f) / cfg->pwm_period;
        break;
    case 4:
        Ta = (T2 + T0 * 0.5f) / cfg->pwm_period;
        Tb = T0 * 0.5f / cfg->pwm_period;
        Tc = (T1 + T2 + T0 * 0.5f) / cfg->pwm_period;
        break;
    case 5:
        Ta = (T1 + T2 + T0 * 0.5f) / cfg->pwm_period;
        Tb = T0 * 0.5f / cfg->pwm_period;
        Tc = (T1 + T0 * 0.5f) / cfg->pwm_period;
        break;
    default:
        Ta = Tb = Tc = 0.0f;
        break;
    }

    // 7. 输出限制到 [0, 1]
    *duty_a = fminf(fmaxf(Ta, 0.0f), 1.0f);
    *duty_b = fminf(fmaxf(Tb, 0.0f), 1.0f);
    *duty_c = fminf(fmaxf(Tc, 0.0f), 1.0f);
}

float unwrap_angle(float new_angle_rad, FOC_State_TypeDef *state)
{
    float delta = new_angle_rad - state->last_raw_angle_rad;

    // 处理跨圈跳变
    if (delta > _PI)
        delta -= _2PI;
    else if (delta < -_PI)
        delta += _2PI;

    state->last_unwrapped_angle_rad += delta;
    state->last_raw_angle_rad = new_angle_rad;

    return state->last_unwrapped_angle_rad;
}

void foc_calibrate_zero_electric_angle(FOC_Instance *foc)
{
    // Step 1: 施加 Ud 电压，Uq = 0，让转子对齐
    for (int i = 0; i < foc->cfg.align_time_ms; ++i)
    {
        float u_d = foc->cfg.align_voltage;

        inv_park_transform(u_d, 0, foc->cfg.alignment_angle, &foc->state.u_alpha,
                           &foc->state.u_beta);

        svpwm_calculate(&foc->cfg, foc->state.u_alpha, foc->state.u_beta,
                        &foc->state.duty_a, &foc->state.duty_b, &foc->state.duty_c);
        foc_set_corrected_pwm(foc);

        foc_delay_ms(foc->cfg.get_micros, 1); // 每毫秒刷新一次
    }
    foc_delay_ms(foc->cfg.get_micros, 200);
    // Step 2: 读取机械角度，转换为电角度
    foc->cfg.zero_electric_angle = normalize_angle_0_to_2pi(
        foc->cfg.pole_pairs * foc->cfg.callbacks.read_angle());
}

void foc_run(FOC_Instance *foc)
{
    if (FOC_MODE_NULL)
        return;

    uint64_t now = foc->cfg.get_micros();
    FOC_State_TypeDef *state = &foc->state;
    FOC_Config_TypeDef *cfg = &foc->cfg;

    switch (cfg->mode)
    {
    case FOC_Mode_Velocity_Openloop:
    {
        float deltaT = (now - state->last_openloop_time_us) * 1e-6f;
        if (deltaT < 0)
            deltaT = 1e-6f;

        state->accumulated_angle_rad +=
            cfg->mode_params.vel_open.target_speed_rad_per_sec * deltaT;
        state->mech_angle_rad = normalize_angle_0_to_2pi(state->accumulated_angle_rad);
        state->last_openloop_time_us = now;
        break;
    }

    case FOC_Mode_Angle_Openloop:
        state->mech_angle_rad = deg_to_rad(cfg->mode_params.angle_open.target_position_deg);
        break;

    default:
        state->mech_angle_rad = cfg->angle_sensor_direction * cfg->callbacks.read_angle();
        break;
    }

    state->electric_angle_rad =
        normalize_angle_0_to_2pi(cfg->pole_pairs * state->mech_angle_rad - cfg->zero_electric_angle);

    speed_calculate(foc);
    state->speed_rad_per_sec = lowpass_filter_update(&cfg->lpf_speed, state->speed_rad_per_sec);

    switch (cfg->mode)
    {
    case FOC_Mode_Velocity_Openloop:
        foc->state.ud = 0;
        foc->state.uq = cfg->mode_params.vel_open.uq_openloop;
        break;

    case FOC_Mode_Angle_Openloop:
        foc->state.ud = 0;
        foc->state.uq = cfg->mode_params.angle_open.uq_openloop;
        break;

    case FOC_Mode_Velocity_Closedloop:
        foc->state.iq = pid_update(&cfg->speed_pid_controller,
                                   cfg->mode_params.vel_closed.target_speed_rad_per_sec,
                                   foc->state.speed_rad_per_sec);
        foc->state.id = 0;
        break;

    case FOC_Mode_Angle_Closedloop:
    {
        float target_angle_rad = deg_to_rad(cfg->mode_params.angle_closed.target_position_deg);
        float position_error = unwrap_angle(state->mech_angle_rad, state) - target_angle_rad;
        float compensated_error = position_error +
                                  cfg->mode_params.angle_closed.angle_closedloop_kv *
                                      state->speed_rad_per_sec;

        float target_speed = pid_update(&cfg->position_pid_controller, 0.0f, compensated_error);
        foc->state.iq = pid_update(&cfg->speed_pid_controller,
                                   target_speed,
                                   foc->state.speed_rad_per_sec);
        foc->state.id = 0;
        break;
    }

    case FOC_Mode_Current_Control:
        foc->state.iq = cfg->mode_params.current.target_iq;
        foc->state.id = cfg->mode_params.current.target_id;
        break;
    default:
        // 不可能进入这里
        return;
    }

    if (cfg->mode == FOC_Mode_Velocity_Closedloop || cfg->mode == FOC_Mode_Angle_Closedloop || cfg->mode == FOC_Mode_Current_Control)
    {
        foc_read_corrected_currents(foc);
        clarke_transform(foc->state.ia, foc->state.ib, foc->state.ic,
                         &foc->state.i_alpha, &foc->state.i_beta);
        park_transform(foc->state.i_alpha, foc->state.i_beta,
                       foc->state.electric_angle_rad,
                       &foc->state.meas_id, &foc->state.meas_iq);

        foc->state.meas_id = lowpass_filter_update(&cfg->lpf_id, foc->state.meas_id);
        foc->state.meas_iq = lowpass_filter_update(&cfg->lpf_iq, foc->state.meas_iq);

        foc->state.uq = pid_update(&cfg->iq_pid_controller, foc->state.iq, foc->state.meas_iq);
        foc->state.ud = pid_update(&cfg->id_pid_controller, foc->state.id, foc->state.meas_id);
    }

    inv_park_transform(foc->state.ud, foc->state.uq,
                       foc->state.electric_angle_rad,
                       &foc->state.u_alpha, &foc->state.u_beta);

    svpwm_calculate(cfg, foc->state.u_alpha, foc->state.u_beta,
                    &foc->state.duty_a, &foc->state.duty_b, &foc->state.duty_c);

    foc_set_corrected_pwm(foc);
}
