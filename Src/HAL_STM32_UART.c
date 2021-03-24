/*
 * HAL_STM32_UART.c
 *
 *  Created on: 9 jul. 2019
 *      Author: TOSHIBA
 */

#include "HAL_STM32_UART.h"

void HAL_STM32_UART_Init(void)
{
	MX_USART1_UART_Init();
}
void HAL_STM32_UART_Transmit(uint8_t* Buffer,uint32_t Len,uint32_t Timeout)
{
	 HAL_UART_Transmit(&huart1,Buffer,Len,Timeout);
}
