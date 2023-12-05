/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
//#include <iostream>
#include <stdio.h>
#include "RFID.hpp"
#include <string>

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
void EnableRxInterrup(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

enum Cases{
	StandBy = 0,
	CmdPorta = 1,
	CmdAC = 2
};

uint8_t recBuff[20] = {0};

volatile bool dataRdyFlag = false;
volatile bool rxIntErrorFlag = false;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART1_UART_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	float temperature = 22.75;

	char ACdata = 'D';
	char DoorData = 'F';
	char tempData[2] = {0};
	uint8_t tempReceive = 0;
	//HAL_UART_Receive_IT(&huart2, recBuff, 20);
	EnableRxInterrup();

	uint8_t Cases = StandBy;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		switch (Cases) {
		case StandBy:
			//ESPERA DADO NA SERIAL
			if (dataRdyFlag) {
				dataRdyFlag = false;
				if (recBuff[0] == '[' && recBuff[7] == ']') {
					ACdata = recBuff[1];
					DoorData = recBuff[6];
					for (int i = 0; i < 2; i++) {
						tempData[i] = recBuff[3 + i];
					}
					tempReceive = (tempData[0] - 48) * 10;
					tempReceive = tempReceive + (tempData[1]) - 48;

				}
			}

			if (rxIntErrorFlag) {
				rxIntErrorFlag = false;
				EnableRxInterrup();
			}
			//VERIFICAÇÃO DOS ESTADOS
			if (ACdata == 'L') {
				Cases = CmdAC;
			} else if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 0 || DoorData == 'A') {
				Cases = CmdPorta;

			} else {
				DoorData = 'F';
				Cases = StandBy;
			}

			break;
		case CmdPorta:
			GPIO_Output(LED_GPIO_Port, LED_Pin, 0);
			HAL_Delay(1000);
			GPIO_Output(LED_GPIO_Port, LED_Pin, 1);
			DoorData = 'F';
			Cases = StandBy;
			break;
		case CmdAC:
			int aux = temperature*100;

			printf("AC Ligado, setado na temperatura de: %d\r\n", tempReceive);
			printf("Temperatura da sala: %f\r\n", temperature);
			char data[6];

			sprintf(data, "[%d]", aux);

			//HAL_UART_Transmit(&huart1, (const uint8_t*)data , 6, HAL_MAX_DELAY); //envia dado para aplicativo

			HAL_UART_Transmit(&huart2, (const uint8_t*)data, 6, HAL_MAX_DELAY); //envia dado para aplicativo
			ACdata = 'D'; //limpa o dado recebido

			Cases = StandBy;
			break;
			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
		}
		/* USER CODE END 3 */
	}
}
	/**
	 * @brief System Clock Configuration
	 * @retval None
	 */
	void SystemClock_Config(void) {
		RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
		RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

		/** Initializes the RCC Oscillators according to the specified parameters
		 * in the RCC_OscInitTypeDef structure.
		 */
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		RCC_OscInitStruct.HSIState = RCC_HSI_ON;
		RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
			Error_Handler();
		}

		/** Initializes the CPU, AHB and APB buses clocks
		 */
		RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
				| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)
				!= HAL_OK) {
			Error_Handler();
		}
	}

	/* USER CODE BEGIN 4 */
	void EnableRxInterrup(void)
	{
		if(HAL_UART_Receive_IT(&huart2, (uint8_t*)recBuff, 8) != HAL_OK)
		{
			rxIntErrorFlag = true;
		}
	}

	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{

		dataRdyFlag = true;
		EnableRxInterrup();
		//HAL_UART_Transmit(&huart1, recBuff, 20, 1000);
		//HAL_UART_Receive_IT(&huart2, recBuff, 20);

	}


	/* USER CODE END 4 */

	/**
	 * @brief  This function is executed in case of error occurrence.
	 * @retval None
	 */
	void Error_Handler(void) {
		/* USER CODE BEGIN Error_Handler_Debug */
		/* User can add his own implementation to report the HAL error return state */
		__disable_irq();
		while (1) {
		}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
