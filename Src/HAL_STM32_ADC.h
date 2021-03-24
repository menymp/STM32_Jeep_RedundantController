/*
 * HAL_STM32_ADC.h
 *
 *  Created on: 9 jul. 2019
 *      Author: TOSHIBA
 */

#ifndef HAL_STM32_ADC_H_
#define HAL_STM32_ADC_H_

#include "adc.h"
//#include "usart.h"
#define HAL_STM32_ADC_Status HAL_StatusTypeDef
#define HAL_ADC_OK 0

void HAL_STM32_ADC_Init(void);
void HAL_STM32_ADC_MeasureStart(void);
HAL_STM32_ADC_Status HAL_STM32_ADC_Poll(uint32_t Timeout);
uint32_t HAL_STM32_ADC_GetVal(void);

#endif /* HAL_STM32_ADC_H_ */
