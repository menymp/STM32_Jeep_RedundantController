/*
 * STM32_C103_CONTROL.c
 *
 *  Created on: 11 jul. 2019
 *      Author: TOSHIBA
 */

#include "HAL_STM32_USB.h"
#include "HAL_STM32_SERVO.h"
#include "HAL_STM32_PWM.h"
#include "HAL_STM32_ADC.h"
#include "HAL_STM32_UART.h"
#include "HAL_STM32_CONTROL.h"

void Motor_set(uint32_t Dir, uint32_t Power)
{
	if(Dir == 0)
	{
		HAL_GPIO_WritePin(MOT_DIR1_PORT,MOT_DIR1_PIN,RESET);
		HAL_GPIO_WritePin(MOT_DIR2_PORT,MOT_DIR2_PIN,RESET);
	}
	if(Dir == 1)
	{
		HAL_GPIO_WritePin(MOT_DIR1_PORT,MOT_DIR1_PIN,SET);
		HAL_GPIO_WritePin(MOT_DIR2_PORT,MOT_DIR2_PIN,RESET);
	}
	if(Dir == 2)
	{
		HAL_GPIO_WritePin(MOT_DIR1_PORT,MOT_DIR1_PIN,RESET);
		HAL_GPIO_WritePin(MOT_DIR2_PORT,MOT_DIR2_PIN,SET);
	}
	HAL_STM32_SetPWMDuty(Power);
}

void Dir_set(uint32_t Angle)
{
	HAL_STM32_ServoPosition(Angle);
}

void Lamp_set(uint32_t Lamp,uint32_t State)
{
	if(Lamp == 0)
	{
		HAL_GPIO_WritePin(LAMP1_PORT,LAMP1_PIN,State);
	}
	if(Lamp == 1)
	{
		HAL_GPIO_WritePin(LAMP2_PORT,LAMP2_PIN,State);
	}
}
