/*
 * HAL_STM32_PWM.c
 *
 *  Created on: 9 jul. 2019
 *      Author: TOSHIBA
 */
#include "HAL_STM32_PWM.h"

void HAL_STM32_InitPWM(void)
{
	MX_TIM3_Init();
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	TIM3->CCR2 = 0;
}
void HAL_STM32_SetPWMDuty(uint32_t Duty)
{
	TIM3->CCR2 = Duty;
}
