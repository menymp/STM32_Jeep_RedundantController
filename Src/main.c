/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
//#include "usbd_cdc_if.h"
#include "HAL_STM32_USB.h"
#include "HAL_STM32_SERVO.h"
#include "HAL_STM32_PWM.h"
#include "HAL_STM32_ADC.h"
#include "HAL_STM32_UART.h"
#include "HAL_STM32_CONTROL.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
extern state State_measure;
//extern UART_HandleTypeDef huart2;
extern volatile float Medicion;
extern volatile int count;
extern volatile int Medicion_d;
/**
  * @brief  The application entry point.
  * @retval int
  */


int main(void)
{
	char Buffer[10];
	uint32_t Len32 = 10;
	//char dataOn[] = "Led On";
	//char dataOff[] = "Led Off";

	uint32_t Adc_val = 0;
	uint8_t Len = 0;
	char BufferTx[11];

	uint32_t Timeout = 0;
	uint32_t Dir = 0;
	uint32_t Pow = 0;

	Sys_state Estado_sistema = SYS_READY;

	Buffer[0] = 'R';
	Buffer[1] = 'D';
	Buffer[2] = 'Y';
	//char dataOk[] = "Dato ok";
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();//sr04
  HAL_STM32_ADC_Init();
  HAL_STM32_UART_Init();
  HAL_STM32_InitServo();//servo init
  HAL_STM32_InitPWM();  //pwm init
  HAL_Delay(100);

  HAL_STM32_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  State_measure = Ready;
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_Delay(100);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(LED_BOARD_PORT,LED_BOARD_PIN,RESET);
  HAL_GPIO_WritePin(LED_BEAT_PORT,LED_BEAT_PIN,SET);
  Lamp_set(0,0);  //LAMP OFF
  Lamp_set(1,0);  //LAMP OFF
  Motor_set(0,0); //00
  Dir_set(1500);  //90
  while (1)
  {
    /* USER CODE END WHILE */
	  switch(Estado_sistema)
	  {
	  case SYS_READY:
		  //espera de instrucciones
		  if(HAL_STM32_VCP_retrieveInputData((uint8_t *)Buffer,&Len32)!=0)
		  {
			  if(Buffer[0] == 'R' && Buffer[1] == 'D' && Buffer[2] == 'Y')
			  {
				//instruccion de inicio
				  Motor_set(0,0); //00
				  Dir_set(1500);  //90
				  Lamp_set(0,0);  //LAMP OFF
				  Lamp_set(1,0);  //LAMP OFF
			  }
			  if(Buffer[0] == 'M' && Buffer[1] == 'O' && Buffer[2] == 'T')
			  {
				//instruccion de motor
				  if(Buffer[3] == 'a') Pow = 0;
				  if(Buffer[3] == 'b') Pow = 1000;
				  if(Buffer[3] == 'c') Pow = 2000;
				  if(Buffer[3] == 'd') Pow = 3000;
				  if(Buffer[3] == 'e') Pow = 4000;

				  if(Buffer[4] == 'a') Motor_set(1,Pow);
				  if(Buffer[4] == 'b') Motor_set(2,Pow);

				  Timeout = 0;
				  Estado_sistema = SYS_BUSY;
			  }
			  if(Buffer[0] == 'D' && Buffer[1] == 'I' && Buffer[2] == 'R')
			  {
				//instruccion de direccion
				  if(Buffer[3] == 'a') Dir = 1000;
				  if(Buffer[3] == 'b') Dir = 1250;
				  if(Buffer[3] == 'c') Dir = 1500;
				  if(Buffer[3] == 'd') Dir = 1750;
				  if(Buffer[3] == 'e') Dir = 2000;
				  Dir_set(Dir);
			  }
			  if(Buffer[0] == 'S' && Buffer[1] == 'N' && Buffer[2] == 'R')
			  {
				//instruccion de ultrasonicos
				  if(State_measure == Ready)HAL_TIM_Base_Start_IT(&htim2);
				  else if(State_measure == Fault) HAL_TIM_Base_Start_IT(&htim2);
				  HAL_Delay(100);
				  Len = sprintf(BufferTx,"%lu\n",(unsigned long)Medicion_d);
				  HAL_STM32_CDC_Transmit_FS((uint8_t *) BufferTx,Len);
			  }
			  if(Buffer[0] == 'L' && Buffer[1] == 'U' && Buffer[2] == 'X')
			  {
				//instruccion de iluminacion
					HAL_STM32_ADC_MeasureStart();
					HAL_Delay(20);
					if(HAL_STM32_ADC_Poll(5) == HAL_ADC_OK)
					{
						Adc_val = HAL_STM32_ADC_GetVal();
						Len = sprintf(BufferTx,"%lu\n",(unsigned long)Adc_val);
						//HAL_STM32_UART_Transmit((uint8_t *)BufferTx,Len,10);
					}
					HAL_STM32_CDC_Transmit_FS((uint8_t *) BufferTx,Len);
			  }
			  if(Buffer[0] == 'L' && Buffer[1] == 'E' && Buffer[2] == 'D')
			  {
				//instruccion de lamparas
				  if(Buffer[3] == 'a' && Buffer[4] == '0') Lamp_set(0,RESET);
				  if(Buffer[3] == 'a' && Buffer[4] == '1') Lamp_set(0,SET);
				  if(Buffer[3] == 'b' && Buffer[4] == '0') Lamp_set(1,RESET);
				  if(Buffer[3] == 'b' && Buffer[4] == '1') Lamp_set(1,SET);
			  }
			  if(Buffer[0] == 'B' && Buffer[1] == 'R' && Buffer[2] == 'K')
			  {
				//instruccion de freno
				  Motor_set(0,0); //00
				  Dir_set(1500);  //90
			  }
		  }
		  break;

	  case SYS_BUSY:
		  Timeout ++;
		  if(HAL_STM32_VCP_retrieveInputData((uint8_t *)Buffer,&Len32)!=0)
		  {
			  if(Buffer[0] == 'R' && Buffer[1] == 'D' && Buffer[2] == 'Y')
			  {
				//instruccion de inicio
				  Motor_set(0,0); //00
				  Dir_set(1500);  //90
				  Lamp_set(0,0);  //LAMP OFF
				  Lamp_set(1,0);  //LAMP OFF
			  }
			  if(Buffer[0] == 'M' && Buffer[1] == 'O' && Buffer[2] == 'T')
			  {
				//instruccion de motor
				  if(Buffer[3] == 'a') Pow = 0;
				  if(Buffer[3] == 'b') Pow = 1000;
				  if(Buffer[3] == 'c') Pow = 2000;
				  if(Buffer[3] == 'd') Pow = 3000;
				  if(Buffer[3] == 'e') Pow = 4000;

				  if(Buffer[4] == 'a') Motor_set(1,Pow);
				  if(Buffer[4] == 'b') Motor_set(2,Pow);

				  Timeout = 0;
				  Estado_sistema = SYS_BUSY;
			  }
			  if(Buffer[0] == 'D' && Buffer[1] == 'I' && Buffer[2] == 'R')
			  {
				//instruccion de direccion
				  if(Buffer[3] == 'a') Dir = 1000;
				  if(Buffer[3] == 'b') Dir = 1250;
				  if(Buffer[3] == 'c') Dir = 1500;
				  if(Buffer[3] == 'd') Dir = 1750;
				  if(Buffer[3] == 'e') Dir = 2000;
				  Dir_set(Dir);
			  }
			  if(Buffer[0] == 'S' && Buffer[1] == 'N' && Buffer[2] == 'R')
			  {
				//instruccion de ultrasonicos
				  if(State_measure == Ready)HAL_TIM_Base_Start_IT(&htim2);
				  else if(State_measure == Fault) HAL_TIM_Base_Start_IT(&htim2);
				  HAL_Delay(100);
				  Len = sprintf(BufferTx,"%lu\n",(unsigned long)Medicion_d);
				  HAL_STM32_CDC_Transmit_FS((uint8_t *) BufferTx,Len);
			  }
			  if(Buffer[0] == 'L' && Buffer[1] == 'U' && Buffer[2] == 'X')
			  {
				//instruccion de iluminacion
					HAL_STM32_ADC_MeasureStart();
					HAL_Delay(20);
					if(HAL_STM32_ADC_Poll(5) == HAL_ADC_OK)
					{
						Adc_val = HAL_STM32_ADC_GetVal();
						Len = sprintf(BufferTx,"%lu\n",(unsigned long)Adc_val);
						//HAL_STM32_UART_Transmit(BufferTx,Len,10);
					}
					HAL_STM32_CDC_Transmit_FS((uint8_t *) BufferTx,Len);
			  }
			  if(Buffer[0] == 'L' && Buffer[1] == 'E' && Buffer[2] == 'D')
			  {
				//instruccion de lamparas
				  if(Buffer[3] == 'a' && Buffer[4] == '0') Lamp_set(0,RESET);
				  if(Buffer[3] == 'a' && Buffer[4] == '1') Lamp_set(0,SET);
				  if(Buffer[3] == 'b' && Buffer[4] == '0') Lamp_set(1,RESET);
				  if(Buffer[3] == 'b' && Buffer[4] == '1') Lamp_set(1,SET);
			  }
			  if(Buffer[0] == 'B' && Buffer[1] == 'R' && Buffer[2] == 'K')
			  {
				//instruccion de freno
				  Motor_set(0,0); //00
				  Dir_set(1500);  //90
			  }
		  }
		  if(Timeout > SYS_TIMEOUT)
		  {
			  //instrucciones de frenado
			  Motor_set(0,0); //00
			  //Dir_set(1500);  //90
			  //Lamp_set(0,0);  //LAMP OFF
			  //Lamp_set(1,0);  //LAMP OFF

			  Timeout = 0;
			  Estado_sistema = SYS_READY;
		  }
		  //instruccion en ejecucion
		  break;

	  case SYS_WAIT:
		  //sistema en pausa
		  break;

	  case SYS_ERROR:
		  //error
		  break;

	  default:
		  break;
	  }
	  HAL_Delay(3);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
