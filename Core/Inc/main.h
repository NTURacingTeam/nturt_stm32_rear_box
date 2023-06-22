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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/*flags for sensor event group*/
static const uint32_t sensorStartEvent = 1U;
static const uint32_t adcTaskCplt = 1U << 1;
static const uint32_t hallTaskCplt = 1U << 2;

/*flags for hall timer proxy*/
static const uint32_t timerLapEvent = 1U;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HALL_FREQ 100
#define R_HAL_Pin GPIO_PIN_0
#define R_HAL_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define R_SUS_Pin GPIO_PIN_4
#define R_SUS_GPIO_Port GPIOA
#define L_SUS_Pin GPIO_PIN_0
#define L_SUS_GPIO_Port GPIOB
#define RTT_SCL_Pin GPIO_PIN_8
#define RTT_SCL_GPIO_Port GPIOA
#define CAN_RX_Pin GPIO_PIN_11
#define CAN_RX_GPIO_Port GPIOA
#define CAN_TX_Pin GPIO_PIN_12
#define CAN_TX_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define LTT_SCL_Pin GPIO_PIN_15
#define LTT_SCL_GPIO_Port GPIOA
#define BRAKE_LIGHT_Pin GPIO_PIN_4
#define BRAKE_LIGHT_GPIO_Port GPIOB
#define RTT_SDA_Pin GPIO_PIN_5
#define RTT_SDA_GPIO_Port GPIOB
#define L_HALL_Pin GPIO_PIN_6
#define L_HALL_GPIO_Port GPIOB
#define L_HALL_EXTI_IRQn EXTI9_5_IRQn
#define LTT_SDA_Pin GPIO_PIN_7
#define LTT_SDA_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
