/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct
{
	char OwnerID[50];
	char SensorID[50];
	char MicrocontrollerID[50];
	char MicrocontrollerPassword[20];
} MC_GeneralInfo;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint16_t GetSensorValue(void);
int ReadDataFromSensor(const char* sensorID, uint16_t* sensorValue);
void WriteSensorData(void);
void FromatSensorValueForWiFi(uint16_t sensorValue, char *result, size_t size);
void CheckRequests();
void SendData(const char *data);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define GasSensor_Pin GPIO_PIN_5
#define GasSensor_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */
#define TactsInOneSecond 1344
#define MinSecondsBetweenSDWrite 60
#define MinSecondsBetweenWifiTransmit 10 //1800
#define MaxSecondsBetweenSDWrite 3195660
#define MaxSecondsBetweenWifiTransmit 3195660
#define TransmitDataLength 200
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
