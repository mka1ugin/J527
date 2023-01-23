/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
  LIN_RECEIVING_BREAK 									    = 0x01,
  LIN_RECEIVING_SYNC    									  = 0x02,
  LIN_RECEIVING_ID      									  = 0x03,
  LIN_RECEIVING_DATA_FROM_MASTER_UART1      = 0x04
} LIN_State;
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
extern uint16_t adcData[1];
extern uint8_t buttonData[9];

void decodeButtonData(void);

extern IWDG_HandleTypeDef hiwdg;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Voltage_ADC_Pin GPIO_PIN_1
#define Voltage_ADC_GPIO_Port GPIOA
#define Heater_Pin GPIO_PIN_2
#define Heater_GPIO_Port GPIOA
#define Assist_Pin GPIO_PIN_3
#define Assist_GPIO_Port GPIOA
#define Cruise_Pin GPIO_PIN_4
#define Cruise_GPIO_Port GPIOA
#define USART1_SLP_Pin GPIO_PIN_11
#define USART1_SLP_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define enableVoltage 12.5
#define disableVoltage 12.0
#define MEAS_NUM	255

#define LIN_ID 		0x8E
#define CRUISE_ID	0x74
#define ASSIST_ID	0x0C
#define HEATER_ID	0x25

#define LIN_SYNC_BYTE	0x55
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
