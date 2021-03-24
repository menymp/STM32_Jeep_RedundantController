/*
 * HAL_STM32_ADC.c
 *
 *  Created on: 9 jul. 2019
 *      Author: TOSHIBA
 */
#include "HAL_STM32_ADC.h"

void HAL_STM32_ADC_Init(void)
{
	MX_ADC1_Init();
}
void HAL_STM32_ADC_MeasureStart(void)
{
	HAL_ADC_Start(&hadc1);
}
HAL_STM32_ADC_Status HAL_STM32_ADC_Poll(uint32_t Timeout)
{
	return HAL_ADC_PollForConversion(&hadc1,Timeout);
}
uint32_t HAL_STM32_ADC_GetVal(void)
{
	return HAL_ADC_GetValue(&hadc1);
}
