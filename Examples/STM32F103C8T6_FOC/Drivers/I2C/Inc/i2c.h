/*
 * i2c.h
 *
 *  Created on: Nov 30, 2024
 *      Author: 20854
 */

#ifndef APP_I2C_H_
#define APP_I2C_H_

#include "stm32f1xx_hal.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
}I2C_Config_TypeDef;

HAL_StatusTypeDef I2C_Write(I2C_Config_TypeDef *i2c_config, uint8_t reg_address, uint8_t* data);
HAL_StatusTypeDef I2C_Read(I2C_Config_TypeDef *i2c_config, uint8_t reg_address, uint8_t* data, uint16_t length);

#endif /* APP_I2C_H_ */
