/*
 * as5600.h
 *
 *  Created on: Jun 23, 2025
 *      Author: Sab1e
 */

#ifndef AS5600_INC_AS5600_H_
#define AS5600_INC_AS5600_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "i2c.h"

#define AS5600_MAX_RANGE     4096  // 12位ADC

#define AS5600_REG_ANGLE     0x0E  // 原始角度寄存器地址

// 方向定义
typedef enum {
    AS5600_CW = 0,   // 顺时针为正
    AS5600_CCW       // 逆时针为正
} AS5600_Direction;

// 状态码
typedef enum {
    AS5600_OK = 0,
    AS5600_ERROR,
    AS5600_NULL_HANDLE
} AS5600_Status;

// AS5600 主结构体
typedef struct {
    I2C_Config_TypeDef i2c;
    AS5600_Direction dir;
    float last_angle;
    float offset_angle;
    int32_t full_rotation_count;
} AS5600_TypeDef;

// API函数
AS5600_Status as5600_init(AS5600_TypeDef *as5600, I2C_HandleTypeDef *hi2c, uint8_t address, AS5600_Direction dir);

float as5600_read_angle_rad(AS5600_TypeDef *as5600);
float as5600_read_angle_deg(AS5600_TypeDef *as5600);
float as5600_read_angle_continuous_rad(AS5600_TypeDef *as5600);
void as5600_set_zero_position(AS5600_TypeDef *as5600);


#endif /* AS5600_INC_AS5600_H_ */
