/*
 * HAL_STM32_SERVO.h
 *
 *  Created on: 9 jul. 2019
 *      Author: TOSHIBA
 */

#ifndef HAL_STM32_SERVO_H_
#define HAL_STM32_SERVO_H_

#include "tim.h"

void HAL_STM32_InitServo(void);
void HAL_STM32_ServoPosition(uint32_t Pos);

#endif /* HAL_STM32_SERVO_H_ */
