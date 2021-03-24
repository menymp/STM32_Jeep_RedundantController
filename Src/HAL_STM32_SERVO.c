/*
 * HAL_STM32_SERVO.c
 *
 *  Created on: 9 jul. 2019
 *      Author: TOSHIBA
 */
#include "HAL_STM32_SERVO.h"

void HAL_STM32_InitServo(void)
{
	MX_TIM4_Init();
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	TIM4->CCR1 = 1000;
}
void HAL_STM32_ServoPosition(uint32_t Pos)
{
	TIM4->CCR1 = Pos;
}

