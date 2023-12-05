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
#include <cstdio>
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "ClockCalendar.h"
#include "RFID.hpp"
#include "AC_controller.hpp"
#include "BMP280.hpp"
#include "List.hpp"
#include "MQTT.hpp"
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
enum cases{
	StandBy = 0,
	CmdPorta,
	CmdAC,
	CommAndroid,
	CmdLog
};

#define MASTER_CARD_UID 0xa3a59670
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint8_t cases = StandBy;
	uint32_t card;
	uint32_t registeredCards[16];
	string acButton;
	string acTempMButton;
	string acTempPButton;
	string doorButton;
	uint8_t flagAcOn = 0;
	uint8_t flagDoorOpen = 0;
	uint8_t flagPlusTemp = 0;
	uint8_t flagMinusTemp = 0;
	float temperature = 0.00;
	const char *ssid = "LISHA_WIFI";
	const char *password = "l1sh4_2022";
	const char *broker = "mqtt.eclipse.org";
	uint16_t mqttPort = 1883;
	const char *topicAcOn = "/acbutton/";
	const char *topicAcTempPlus = "/acplustemp/";
	const char *topicAcTempMinus = "/acminustemp/";
	const char *topic_out = "/temperature/";
	const char *topicDoorButton = "/doorButton/";

	uint8_t temp = 24; //valro padrão de temperatura do ar condicionado
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
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	ClockCalendar clkCalendar(9, 20, 2023, 7, 32, 20, 1); // 13/09/2023 - 7:50:50 PM;
	RFID rfid(CS_GPIO_Port, CS_Pin);
	rfid.RFID_Init();
	BMP280 bmp(0xF5, 0xF4);
	AC_controller ac(*INFRARED_GPIO_Port, INFRARED_Pin);
	List log;
	MQTT mqtt(broker, mqttPort, ssid, password);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		switch (cases) {
		case StandBy:
			acButton = mqtt.Subscribe(topicAcOn, 2);
			acTempMButton = mqtt.Subscribe(topicAcTempMinus, 1);
			acTempPButton = mqtt.Subscribe(topicAcTempPlus, 1);
			doorButton = mqtt.Subscribe(topicDoorButton, 2);

			temperature = bmp.readSensor();

			if(acButton == "L")
			{
				flagAcOn = 1;
				cases = CommAndroid;
			}
			else if (acTempMButton == "M"){
				flagMinusTemp = 1;
				cases = CommAndroid;
			}
			else if (acTempPButton == "P"){
				flagPlusTemp = 1;
				cases = CommAndroid;
			}
			else if (doorButton == "L"){
				flagDoorOpen = 1;
				cases = CommAndroid;
			}

			else{
				flagAcOn = 0;
				flagDoorOpen = 0;
				flagMinusTemp = 0;
				flagPlusTemp = 0;
				cases = StandBy;

			}

			int hour, minute, second, isPM, day, month, year;
			temperature = bmp.readSensor();
			card = rfid.readSensor();
			for (uint8_t i = 0; i < 16; i++) {
				if (card == registeredCards[i]) {
					cases = CmdPorta;
					card = 0;
				} else if (card == MASTER_CARD_UID) {
					delay_us(1000);
					uint32_t temp_card = rfid.readSensor();
					if (registeredCards[i] == 0) {
						registeredCards[i] = temp_card;
						cases = CmdPorta;
					}
				}
			}

			break;
		case CmdPorta:
			GPIO_Output(DOOR_GPIO_Port, DOOR_Pin, 1);
			delay_us(1000);
			GPIO_Output(DOOR_GPIO_Port, DOOR_Pin, 0);
			clkCalendar.readClock(hour, second, minute, isPM);
			log.insertAfterLast(rfid.getSensorID(), temp, 1, flagAcOn, hour, minute, second, isPM);
			cases = StandBy;
			break;
		case CmdAC:
			switch (temp) {
			case 16:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T16, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 17:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T17, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 18:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T18, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 19:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T19, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 20:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T20, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 21:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T21, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 22:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T22, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 23:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T23, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 24:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T24, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 25:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T25, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 26:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T26, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 27:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T27, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 28:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T28, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 29:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T29, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 30:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T30, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 31:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T31, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			case 32:
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T32, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			default: //liga o ar e seta em 24
				ac.sendCommand(AC_controller::COOL, AC_controller::FAN_AUTO, AC_controller::T24, AC_controller::TURBO_ON, AC_controller::SWING_ON, AC_controller::SHEET_ON);
				log.insertAfterLast(rfid.getSensorID(), temp, 0, flagAcOn, hour, minute, second, isPM);
				break;
			}
			cases = StandBy;
			break;

		case CommAndroid:
			if(flagAcOn) cases = CmdAC;
			else if (flagDoorOpen) cases = CmdPorta;
			else if(flagMinusTemp){
				temp--;
				cases = CmdAC;
			}
			else if (flagPlusTemp){
				temp++;
				cases = CmdAC;
			}

			break;
		case CmdLog:
			int bmpTemp = temperature*1000;
			char sendTemp[10];
			sprintf(sendTemp, "%d", bmpTemp);
			mqtt.Publish(topic_out, sendTemp);

			int	T[8] = {0}; //buffer transmissao log
			char sendLog[100];
			for (uint8_t numNodes = 0; numNodes < log.listAll(); numNodes++) {
				log.removeFirst(T[0], T[1], T[2], T[3], T[4], T[5], T[6], T[7]);
				sprintf(sendLog, "%d", T[numNodes]);
				mqtt.Publish(topic_out, sendLog);
				T[numNodes] = 0; //limpa o buffer;
			}
			break;

//		default:
//			break;
		}

		clkCalendar.advance();
		HAL_Delay(1000);
		/* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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
  __disable_irq();
  while (1)
  {
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
