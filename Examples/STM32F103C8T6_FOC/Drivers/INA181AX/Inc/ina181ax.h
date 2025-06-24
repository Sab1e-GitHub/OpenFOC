/*
 * ina181ax.h
 *
 *  Created on: Jun 23, 2025
 *      Author: Sab1e
 */

#ifndef INA181AX_INC_INA181AX_H_
#define INA181AX_INC_INA181AX_H_

#include "adc.h"

typedef enum {
    INA181AX_OK = 0,
    INA181AX_ERR = -1,
    INA181AX_NULL_HANDLE = -2
} INA181AX_Status;

typedef enum {
    INA181A1 = 0, // 增益20
    INA181A2 = 1, // 增益50
    INA181A3 = 2, // 增益100
    INA181A4 = 3  // 增益200
} INA181AX_Chip;

typedef struct {
    float r_shunt;             // 分流电阻
    float gain;                // INA181AX 放大倍数
    GPIO_TypeDef *gpio_port;     // EN 控制引脚
    uint16_t gpio_pin;
    ADC_Config_TypeDef *adc;     // ADC 配置结构体
    float v_offset;            // 零点偏移电压
    float i_filtered;          // 滤波后的电流
    float v_ref;               // ADC参考电压
} INA181AX_TypeDef;

INA181AX_Status ina181ax_init(INA181AX_TypeDef *sensor,
                                      float r_shunt,
                                      float v_ref,
                                      INA181AX_Chip chip,
                                      GPIO_TypeDef *gpio_port,
                                      uint16_t gpio_pin,
                                      ADC_Config_TypeDef *adc_config);

INA181AX_Status ina181ax_calibrate(INA181AX_TypeDef *sensor);

INA181AX_Status ina181ax_read_current(INA181AX_TypeDef *sensor,
                                              float *current_out);

INA181AX_Status ina181ax_read_current_filtered(INA181AX_TypeDef *sensor,
                                                       float *current_out,
                                                       uint8_t sample_count,
                                                       float alpha);

#endif /* INA181AX_INC_INA181AX_H_ */
