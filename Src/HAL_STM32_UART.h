/*
 * HAL_STM32_UART.h
 *
 *  Created on: 9 jul. 2019
 *      Author: TOSHIBA
 */

#ifndef HAL_STM32_UART_H_
#define HAL_STM32_UART_H_

#include "usart.h"

void HAL_STM32_UART_Init(void);
void HAL_STM32_UART_Transmit(uint8_t* Buffer,uint32_t Len,uint32_t Timeout);

#endif /* HAL_STM32_UART_H_ */
