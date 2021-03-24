/*
 * HAL_STM32_CONTROL.h
 *
 *  Created on: 11 jul. 2019
 *      Author: TOSHIBA
 */

#ifndef HAL_STM32_CONTROL_H_
#define HAL_STM32_CONTROL_H_

typedef enum System_state
{
	SYS_WAIT,
	SYS_BUSY,
	SYS_ERROR,
	SYS_READY
}Sys_state;



#define SYS_TIMEOUT 	70

#define LED_BOARD_PIN	GPIO_PIN_13
#define LED_BOARD_PORT	GPIOC

#define LED_BEAT_PIN	GPIO_PIN_8
#define LED_BEAT_PORT	GPIOA

#define MOT_DIR1_PIN	GPIO_PIN_6//5
#define MOT_DIR1_PORT	GPIOA

#define MOT_DIR2_PIN	GPIO_PIN_5//5
#define MOT_DIR2_PORT	GPIOA

#define LAMP1_PIN		GPIO_PIN_1
#define LAMP1_PORT		GPIOB

#define LAMP2_PIN		GPIO_PIN_0
#define LAMP2_PORT		GPIOB


void Motor_set(uint32_t Dir, uint32_t Power);
void Dir_set(uint32_t Angle);
void Lamp_set(uint32_t Lamp,uint32_t State);

#endif /* HAL_STM32_CONTROL_H_ */
