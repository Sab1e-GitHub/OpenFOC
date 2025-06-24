/*
 * ina181ax.c
 *
 *  Created on: Jun 23, 2025
 *      Author: Sab1e
 */

#include "ina181ax.h"
#include <math.h>

INA181AX_Status ina181ax_calibrate(INA181AX_TypeDef *sensor) {
    if (!sensor || !sensor->adc)
        return INA181AX_NULL_HANDLE;

    float sum = 0.0f;
    const uint16_t read_times = 1000;

    for (uint16_t i = 0; i < read_times; ++i) {
        uint32_t adc_val = 0;
        if (adc_read(sensor->adc, &adc_val) != ADC_OK)
            return INA181AX_ERR;

        float v_out = ((float)adc_val / (float)sensor->adc->max) * sensor->v_ref;
        sum += v_out;
        HAL_Delay(1); // 使用 Cube HAL 的延迟函数
    }

    sensor->v_offset = sum / (float)read_times;
    return INA181AX_OK;
}

INA181AX_Status ina181ax_init(INA181AX_TypeDef *sensor,
                                      float r_shunt,
                                      float v_ref,
                                      INA181AX_Chip chip,
                                      GPIO_TypeDef *gpio_port,
                                      uint16_t gpio_pin,
                                      ADC_Config_TypeDef *adc_config) {
    if (!sensor || !adc_config || r_shunt <= 0.0f)
        return INA181AX_NULL_HANDLE;

    sensor->r_shunt = r_shunt;
    sensor->v_ref = v_ref;
    sensor->gpio_port = gpio_port;
    sensor->gpio_pin = gpio_pin;
    sensor->i_filtered = 0.0f;
    sensor->adc = adc_config;

    switch (chip) {
        case INA181A1: sensor->gain = 20.0f; break;
        case INA181A2: sensor->gain = 50.0f; break;
        case INA181A3: sensor->gain = 100.0f; break;
        case INA181A4: sensor->gain = 200.0f; break;
        default: return INA181AX_ERR;
    }

    if (ina181ax_calibrate(sensor) != INA181AX_OK)
        return INA181AX_ERR;

    return INA181AX_OK;
}

INA181AX_Status ina181ax_read_current(INA181AX_TypeDef *sensor,
                                              float *current_out) {
    if (!sensor || !sensor->adc || !current_out)
        return INA181AX_NULL_HANDLE;

    uint32_t adc_raw = 0;
    if (adc_read(sensor->adc, &adc_raw) != ADC_OK)
        return INA181AX_ERR;

    float v_out = ((float)adc_raw / (float)sensor->adc->max) * sensor->v_ref;
    float v_diff = v_out - sensor->v_offset;
    *current_out = v_diff / (sensor->r_shunt * sensor->gain);

    return INA181AX_OK;
}

INA181AX_Status ina181ax_read_current_filtered(INA181AX_TypeDef *sensor,
                                                       float *current_out,
                                                       uint8_t sample_count,
                                                       float alpha) {
    if (!sensor || !sensor->adc || !current_out || sample_count == 0 || alpha <= 0.0f || alpha > 1.0f)
        return INA181AX_ERR;

    float current_avg = 0.0f;

    for (uint8_t i = 0; i < sample_count; ++i) {
        uint32_t adc_val = 0;
        if (adc_read(sensor->adc, &adc_val) != ADC_OK)
            return INA181AX_ERR;

        float v_out = ((float)adc_val / (float)sensor->adc->max) * sensor->v_ref;
        float v_diff = v_out - sensor->v_offset;
        float current_sample = v_diff / (sensor->r_shunt * sensor->gain);
        if (fabsf(current_sample) < 0.01f) current_sample = 0.0f;
        current_avg += current_sample;
    }

    current_avg /= sample_count;

    // 一阶低通滤波
    sensor->i_filtered = alpha * current_avg + (1.0f - alpha) * sensor->i_filtered;
    *current_out = sensor->i_filtered;

    return INA181AX_OK;
}
