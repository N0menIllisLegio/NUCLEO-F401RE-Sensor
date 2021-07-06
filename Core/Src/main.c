/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include <string.h>
#include <stdio.h>
#include "MicroSD.h"
#include "wifi.h"

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
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};

int ServerQueriesSocketID = -1;
int GasSensorSocketID = -1;

// ---------------------------------------
// ------------ CONFIG VARS --------------
// ---------------------------------------

WiFi_GeneralInfo wifi_info;

// ---------------------------------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void LoadConfigs(void);

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  LoadConfigs();
  WiFi_Connect(wifi_info.SSID, wifi_info.SecKey, wifi_info.PrivMode);

	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 21;
  sTime.Minutes = 40;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_SEPTEMBER;
  sDate.Date = 14;
  sDate.Year = 20;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 62500;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 6720;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 62500;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 13440;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 62500;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 26880;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 62500;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 13440;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint32_t ParseIntParameter(char *buffer, const char *param, uint32_t defaultValue)
{
	char *parameterStr = strstr(buffer, param);
	uint8_t pointer = strlen(param);
	uint32_t conversionError = 0;
	uint32_t result = 0;

	while(parameterStr[pointer] != ';' && conversionError == 0)
	{
		if(parameterStr[pointer] >= '0' && parameterStr[pointer] <= '9')
		{
			result = (result * 10) + (parameterStr[pointer++] - '0');
		}
		else
		{
			conversionError = 1;
		}
	}

	if(conversionError == 1)
	{
		return defaultValue;
	}

	return result;
}

void ParseStrParameter(char *buffer, const char *param, char *result, size_t arraySize)
{
	char *parameterStr = strstr(buffer, param);
	uint8_t pointer = strlen(param);
	memccpy(result, parameterStr + pointer, ';', arraySize);
	result[strlen(result) - 1] = '\0';
}

void LoadConfigs(void)
{
	char buffer[1000];
	UINT bytesRead;

	ReadConfigs(buffer, &bytesRead);

	sDate.Date = ParseIntParameter(buffer, "Date=", 1);
	sDate.Month = ParseIntParameter(buffer, "Month=", 1);
	sDate.Year = ParseIntParameter(buffer, "Year=", 70);
	sDate.WeekDay = ParseIntParameter(buffer, "WeekDay=", 4);

	sTime.Hours = ParseIntParameter(buffer, "Hours=", 0);
	sTime.Minutes = ParseIntParameter(buffer, "Minutes=", 0);
	sTime.Seconds = ParseIntParameter(buffer, "Seconds=", 0);

	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	wifi_info.Port = ParseIntParameter(buffer, "Port=", 0);
	wifi_info.PrivMode = ParseIntParameter(buffer, "PrivateMode=", 0);
	ParseStrParameter(buffer, "SSID=", wifi_info.SSID, sizeof(wifi_info.SSID));
	ParseStrParameter(buffer, "SecurityKey=", wifi_info.SecKey, sizeof(wifi_info.SecKey));
	ParseStrParameter(buffer, "IP=", wifi_info.IP, sizeof(wifi_info.IP));
	ParseStrParameter(buffer, "Protocol=", wifi_info.Protocol, sizeof(wifi_info.Protocol));

	ParseStrParameter(buffer, "OwnerID=", mc_info.OwnerID, sizeof(mc_info.OwnerID));
	ParseStrParameter(buffer, "MicrocontrollerID=", mc_info.MicrocontrollerID, sizeof(mc_info.MicrocontrollerID));
	ParseStrParameter(buffer, "SensorID=", mc_info.SensorID, sizeof(mc_info.SensorID));
	ParseStrParameter(buffer, "MicrocontrollerPassword=", mc_info.MicrocontrollerPassword, sizeof(mc_info.MicrocontrollerPassword));

	int writeSDIntervalSeconds = ParseIntParameter(buffer, "writeSDIntervalSeconds=", MinSecondsBetweenSDWrite);
	int transmitIntervalSeconds = ParseIntParameter(buffer, "transmitIntervalSeconds=", MinSecondsBetweenWifiTransmit);
	int requestIntervalSeconds = ParseIntParameter(buffer, "requestIntervalSeconds=", MinSecondsBetweenWifiRequest);

	if(writeSDIntervalSeconds > MaxSecondsBetweenSDWrite)
	{
		writeSDIntervalSeconds = MaxSecondsBetweenSDWrite;
	}
	else if (writeSDIntervalSeconds < MinSecondsBetweenSDWrite)
	{
		writeSDIntervalSeconds = MinSecondsBetweenSDWrite;
	}

	if(transmitIntervalSeconds > MaxSecondsBetweenWifiTransmit)
	{
		transmitIntervalSeconds = MaxSecondsBetweenWifiTransmit;
	}
	else if (transmitIntervalSeconds < MinSecondsBetweenWifiTransmit)
	{
		transmitIntervalSeconds = MinSecondsBetweenWifiTransmit;
	}

	if(requestIntervalSeconds > MaxSecondsBetweenWifiRequest)
	{
		requestIntervalSeconds = MaxSecondsBetweenWifiRequest;
	}
	else if (requestIntervalSeconds < MinSecondsBetweenWifiRequest)
	{
		requestIntervalSeconds = MinSecondsBetweenWifiRequest;
	}

	htim1.Instance->ARR = writeSDIntervalSeconds * TactsInOneSecond;
	htim2.Instance->ARR = transmitIntervalSeconds * TactsInOneSecond;
	htim3.Instance->ARR = requestIntervalSeconds * TactsInOneSecond;
}

uint16_t GetSensorValue(void)
{
	uint16_t sensorValue = 0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	sensorValue = HAL_ADC_GetValue(&hadc1);

	return sensorValue;
}

void WriteSensorData(const char* sensorID)
{
	HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);

	uint16_t sensorValue;
	if (ReadDataFromSensor(sensorID, &sensorValue) == 1)
	{
		char line[100];

		snprintf(line, 99, "%s;%d:%d:%d;%d\n", sensorID, sTime.Hours, sTime.Minutes, sTime.Seconds, sensorValue);

		WriteFile(line);
	}

	HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
}

void FromatSensorValueForWiFi(uint16_t sensorValue, char *result, size_t size)
{
	RTC_TimeTypeDef time = {0};
	RTC_DateTypeDef date = {0};

	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

	snprintf(result, size, "%s|%s;%d/%d/%d %d:%d:%d;%d", SERVER_DATA,
		mc_info.SensorID,
		date.Month, date.Date, date.Year, time.Hours, time.Minutes, time.Seconds,
		sensorValue);
}

void SendData(const char *data)
{
	WiFi_Status_t wifi_status = WiFi_MODULE_SUCCESS;

	if(WiFi_PingServer(wifi_info.IP) != WiFi_MODULE_SUCCESS)
	{
		wifi_status = WiFi_Connect(wifi_info.SSID, wifi_info.SecKey, wifi_info.PrivMode);
	}

	if(wifi_status == WiFi_MODULE_SUCCESS)
	{
		if (GasSensorSocketID != -1)
		{
			Socket_Close(&GasSensorSocketID);
		}

		GasSensorSocketID = Socket_Connect(wifi_info.IP, wifi_info.Port, wifi_info.Protocol);

		if(GasSensorSocketID != -1)
		{
			char receivedData[10];

			if(Socket_TransmitData(GasSensorSocketID, data) == Socket_SUCCESS)
			{
				Socket_ReadData(GasSensorSocketID, receivedData);
			}

			Socket_Close(&ServerQueriesSocketID);
		}
	}
}

void CheckRequests()
{
	WiFi_Status_t wifi_status = WiFi_MODULE_SUCCESS;

	if(WiFi_PingServer(wifi_info.IP) != WiFi_MODULE_SUCCESS)
	{
		wifi_status = WiFi_Connect(wifi_info.SSID, wifi_info.SecKey, wifi_info.PrivMode);
	}

	if(wifi_status == WiFi_MODULE_SUCCESS)
	{
		if (ServerQueriesSocketID != -1)
		{
			Socket_Close(&ServerQueriesSocketID);
		}

		ServerQueriesSocketID = Socket_Connect(wifi_info.IP, wifi_info.Port, wifi_info.Protocol);

		if(ServerQueriesSocketID != -1)
		{
			Socket_Status_t status = Socket_TransmitData(ServerQueriesSocketID, SERVER_REQUEST);

			if(status == Socket_SUCCESS)
			{
				char receivedData[47];
				Socket_ReadData(ServerQueriesSocketID, receivedData);

				if(strstr(receivedData, SERVER_SV_RESP) != NULL)
				{
					char queriedSensorID[37];
					ParseStrParameter(receivedData, "Server_SV|", queriedSensorID, sizeof(queriedSensorID));
					queriedSensorID[36] = '\0';

					uint16_t sensorValue;
					if (ReadDataFromSensor(queriedSensorID, &sensorValue) == 1)
					{
						char sensorData[TransmitDataLength];
						FromatSensorValueForWiFi(sensorValue, sensorData, TransmitDataLength);

						char receivedData[10];

						if(Socket_TransmitData(ServerQueriesSocketID, sensorData) == Socket_SUCCESS)
						{
							Socket_ReadData(ServerQueriesSocketID, receivedData);
						}
					}
				}
			}

			Socket_Close(&ServerQueriesSocketID);
		}
	}
}

int ReadDataFromSensor(const char* sensorID, uint16_t* sensorValue)
{
	if (strcasecmp(sensorID, mc_info.SensorID) == 0)
	{
		*sensorValue = GetSensorValue();
		return 1;
	}

	return 0;
}

void SynchronizeDateTime()
{
	WiFi_Status_t wifi_status = WiFi_MODULE_SUCCESS;

	if(WiFi_PingServer(wifi_info.IP) != WiFi_MODULE_SUCCESS)
	{
		wifi_status = WiFi_Connect(wifi_info.SSID, wifi_info.SecKey, wifi_info.PrivMode);
	}

	if(wifi_status == WiFi_MODULE_SUCCESS)
	{
		int dateTimeSocketID = Socket_Connect(wifi_info.IP, wifi_info.Port, wifi_info.Protocol);

		if(dateTimeSocketID != -1)
		{
			Socket_Status_t status;
			char receivedData[80];
			char sendData[25];

			snprintf(sendData, sizeof(sendData), "%s|%d/%d/%d %d:%d:%d", STM_DATETIME,
				sDate.Month, sDate.Date, sDate.Year,
				sTime.Hours, sTime.Minutes, sTime.Seconds);

			status = Socket_TransmitData(dateTimeSocketID, sendData);

			if(status == Socket_SUCCESS)
			{
				Socket_ReadData(dateTimeSocketID, receivedData);

				if (strstr(receivedData, SERVER_DT_RESP) != NULL)
				{
					sDate.Date = ParseIntParameter(receivedData, "Date=", 1);
					sDate.Month = ParseIntParameter(receivedData, "Month=", 1);
					sDate.Year = ParseIntParameter(receivedData, "Year=", 70);
					sDate.WeekDay = ParseIntParameter(receivedData, "WeekDay=", 4);

					sTime.Hours = ParseIntParameter(receivedData, "Hours=", 0);
					sTime.Minutes = ParseIntParameter(receivedData, "Minutes=", 0);
					sTime.Seconds = ParseIntParameter(receivedData, "Seconds=", 0);

					HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
					HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

					HAL_TIM_Base_Stop_IT(&htim4);
				}
			}

			Socket_Close(&dateTimeSocketID);
		}
	}
}

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
