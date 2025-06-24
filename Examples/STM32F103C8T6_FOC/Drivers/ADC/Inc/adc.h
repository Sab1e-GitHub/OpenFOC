/*
 * adc.h
 *
 *  Created on: Jun 23, 2025
 *      Author: Sab1e
 */

#ifndef ADC_INC_ADC_H_
#define ADC_INC_ADC_H_

#include "stm32f1xx_hal.h"

typedef struct {
	ADC_HandleTypeDef *hadc;
	uint32_t channel;
	uint32_t sampling_time;
	uint32_t max;
} ADC_Config_TypeDef;

typedef enum {
	ADC_OK = 0,
	ADC_ERR = -1,
	ADC_NULL_HANDLE = -2,
	ADC_BUSY = -3,
	ADC_TIMEOUT = -4
} ADC_Status_TypeDef;

ADC_Status_TypeDef adc_init(ADC_Config_TypeDef *adc, ADC_HandleTypeDef *hadc,
				uint32_t channel,uint32_t max, uint32_t sampling_time);
ADC_Status_TypeDef adc_read(ADC_Config_TypeDef *adc, uint32_t *output);
#endif /* ADC_INC_ADC_H_ */
