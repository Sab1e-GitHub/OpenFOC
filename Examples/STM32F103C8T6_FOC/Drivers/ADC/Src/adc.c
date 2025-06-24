/*
 * adc.c
 *
 *  Created on: Jun 23, 2025
 *      Author: Sab1e
 */
#include "adc.h"

ADC_Status_TypeDef adc_init(ADC_Config_TypeDef *adc, ADC_HandleTypeDef *hadc,
				uint32_t channel,uint32_t max, uint32_t sampling_time) {
	if (adc == NULL || hadc == NULL) {
		return ADC_NULL_HANDLE;
	}

	adc->hadc = hadc;
	adc->channel = channel;
	adc->sampling_time = sampling_time;
	adc->max = max;

	return ADC_OK;
}

ADC_Status_TypeDef adc_read(ADC_Config_TypeDef *adc, uint32_t *output) {
	if (adc == NULL || adc->hadc == NULL || output == NULL) {
		return ADC_NULL_HANDLE;
	}

	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = adc->channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = adc->sampling_time;

	if (HAL_ADC_ConfigChannel(adc->hadc, &sConfig) != HAL_OK) {
		return ADC_ERR;
	}

	if (HAL_ADC_Start(adc->hadc) != HAL_OK) {
		return ADC_BUSY;
	}

	if (HAL_ADC_PollForConversion(adc->hadc, HAL_MAX_DELAY) != HAL_OK) {
		HAL_ADC_Stop(adc->hadc);
		return ADC_TIMEOUT;
	}

	*output = HAL_ADC_GetValue(adc->hadc);
	HAL_ADC_Stop(adc->hadc);

	return ADC_OK;
}
