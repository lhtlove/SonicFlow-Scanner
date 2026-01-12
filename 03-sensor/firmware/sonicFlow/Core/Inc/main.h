/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RECV_EN_Pin GPIO_PIN_13
#define RECV_EN_GPIO_Port GPIOC
#define RECV_A0_Pin GPIO_PIN_14
#define RECV_A0_GPIO_Port GPIOC
#define RECV_A1_Pin GPIO_PIN_15
#define RECV_A1_GPIO_Port GPIOC
#define OPAMP_IN_Pin GPIO_PIN_1
#define OPAMP_IN_GPIO_Port GPIOA
#define CAN_SPEED_Pin GPIO_PIN_11
#define CAN_SPEED_GPIO_Port GPIOB
#define FG_FCK_Pin GPIO_PIN_13
#define FG_FCK_GPIO_Port GPIOB
#define FG_FSYNC_Pin GPIO_PIN_14
#define FG_FSYNC_GPIO_Port GPIOB
#define FG_SPI_Pin GPIO_PIN_15
#define FG_SPI_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_11
#define CAN_RX_GPIO_Port GPIOA
#define CAN_TX_Pin GPIO_PIN_12
#define CAN_TX_GPIO_Port GPIOA
#define SEL_IN2_Pin GPIO_PIN_4
#define SEL_IN2_GPIO_Port GPIOB
#define SEL_IN3_Pin GPIO_PIN_5
#define SEL_IN3_GPIO_Port GPIOB
#define SEL_IN4_Pin GPIO_PIN_6
#define SEL_IN4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
