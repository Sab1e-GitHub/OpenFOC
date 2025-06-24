/*
 * as5600.c
 *
 *  Created on: Jun 23, 2025
 *      Author: Sab1e
 */


#include "as5600.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265359f
#endif

#ifndef TWO_PI
#define TWO_PI 6.28318530718f
#endif

AS5600_Status as5600_init(AS5600_TypeDef *as5600, I2C_HandleTypeDef *hi2c, uint8_t address, AS5600_Direction dir) {
    if (!as5600 || !hi2c) return AS5600_NULL_HANDLE;

    as5600->i2c.hi2c = hi2c;
    as5600->i2c.address = address;
    as5600->dir = dir;

    float angle = as5600_read_angle_rad(as5600);
    as5600->offset_angle = angle;
    as5600->last_angle = 0.0f;
    as5600->full_rotation_count = 0;

    return AS5600_OK;
}

float as5600_read_angle_deg(AS5600_TypeDef *as5600) {
    if (!as5600 || !as5600->i2c.hi2c) return 0.0f;
    uint8_t data[2] = {0};

    if (I2C_Read(&as5600->i2c, AS5600_REG_ANGLE, data, 2) != HAL_OK)
        return 0.0f;

    uint16_t raw_angle = ((data[0] << 8) | data[1]) & 0x0FFF;
    float angle = (raw_angle * 360.0f) / AS5600_MAX_RANGE;

    if (as5600->dir == AS5600_CCW)
        return fmodf(360.0f - angle, 360.0f);

    return angle;
}

float as5600_read_angle_rad(AS5600_TypeDef *as5600) {
    if (!as5600 || !as5600->i2c.hi2c) return 0.0f;
    uint8_t data[2] = {0};

    if (I2C_Read(&as5600->i2c, AS5600_REG_ANGLE, data, 2) != HAL_OK)
        return 0.0f;

    uint16_t raw_angle = ((data[0] << 8) | data[1]) & 0x0FFF;
    float angle = (raw_angle * TWO_PI) / AS5600_MAX_RANGE;

    if (as5600->dir == AS5600_CCW)
        return fmodf(TWO_PI - angle, TWO_PI);

    return angle;
}

float as5600_read_angle_continuous_rad(AS5600_TypeDef *as5600) {
    if (!as5600 || !as5600->i2c.hi2c) return 0.0f;
    uint8_t data[2] = {0};

    if (I2C_Read(&as5600->i2c, AS5600_REG_ANGLE, data, 2) != HAL_OK)
        return as5600->last_angle + as5600->full_rotation_count * TWO_PI - as5600->offset_angle;

    uint16_t raw_angle = ((data[0] << 8) | data[1]) & 0x0FFF;
    float angle = (raw_angle * TWO_PI) / AS5600_MAX_RANGE;

    if (as5600->dir == AS5600_CCW)
        angle = fmodf(TWO_PI - angle, TWO_PI);

    float delta = angle - as5600->last_angle;
    if (delta > PI) delta -= TWO_PI;
    else if (delta < -PI) delta += TWO_PI;

    if (delta > PI * 0.8f) {
        as5600->full_rotation_count -= 1;
    } else if (delta < -PI * 0.8f) {
        as5600->full_rotation_count += 1;
    }

    as5600->last_angle = angle;

    const int32_t max_rot = 1000;
    if (as5600->full_rotation_count >= max_rot || as5600->full_rotation_count <= -max_rot) {
        as5600->offset_angle += as5600->full_rotation_count * TWO_PI;
        as5600->full_rotation_count = 0;
    }

    return (angle + as5600->full_rotation_count * TWO_PI) - as5600->offset_angle;
}

void as5600_set_zero_position(AS5600_TypeDef *as5600) {
    float angle_now = as5600_read_angle_continuous_rad(as5600);
    as5600->offset_angle = angle_now;
    as5600->full_rotation_count = 0;
    as5600->last_angle = 0.0f;
}
