/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
//
//M_PI

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stepper_setting.hpp"
#include "motors.hpp"
#include "can.hpp"
#include "encoders.hpp"
#include "smc.hpp"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SYSCLK 64000000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint16_t E1_last = 0;
uint16_t E2_last = 0;

uint32_t speedLE = 0, speedRE = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

#define MT6701_ADDR (0x06 << 1)  // Default slave address 0x06 shifted for HAL
#define REG_ANGLE_HIGH 0x03
#define REG_ANGLE_LOW 0x04

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t speedTim(uint8_t id) {
	return ((id ? speedRE : speedLE) > __HAL_TIM_GET_COUNTER(&htim2) ?
			__HAL_TIM_GET_COUNTER(&htim2) + 65536 :
			__HAL_TIM_GET_COUNTER(&htim2)) - (id ? speedRE : speedLE);
}

void separate_float(float value, uint8_t* integer_part, uint8_t* decimal_part) {
    // Extract integer part
    *integer_part = (uint8_t)value;

    // Extract 2 decimal digits
    float decimal_temp = (value - (int)value) * 100;
    *decimal_part = (uint8_t)decimal_temp;
}
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
  MX_I2C1_Init();
  MX_CAN_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);

  stpSetting_init(&huart3);
  gconfSetup(global_config_);
  cconfSetup(chopper_config_, 0b0011);
  currentSetup(driver_current_, 0, current);

  Stepper_Init(&htim1);
  Stepper_SetSpeed(30);

  DC_Init(&htim3, 0);
  DC_Init(&htim4, 1);

  CAN_INIT(&hcan, DEVICE_ID, MASK_SINGLE);

  /* USER CODE END 2 */

//	  stepper_angle = stepper_encoder_angle(&hi2c1, RAD);
  uint8_t E1A, E1B, E2A, E2B;
  uint8_t curE1, curE2, lastE1 = 0, lastE2 = 0;
  uint8_t replyData[8];

  uint64_t start;
  uint8_t dc_flag = 0;

//  static uint16_t enc_div = 0;
//  uint16_t enc_before = 0;
//  uint8_t on_run = 0;

  while (1) {
//	if (++enc_div >= 0) {
//		enc_div = 0;
//
//		stepper_encoder_angle(DEG);
//		increment = abs(raw_angle - enc_before);
//
//		if (stepper.is_running) on_run = 1;
//		else if (on_run && increment <= 10) {
//			Stepper_Stop();
//			on_run = 0;
//
//			replyData[0] = 0x30;
//			packFloat(404.404, replyData + 1);
//			CAN_Tx(replyData, 8, DEVICE_MASTER);
//		} else if (on_run) {
//			replyData[0] = 0x30;
//			packFloat(increment, replyData + 1);
//			CAN_Tx(replyData, 8, DEVICE_MASTER);
//		}
////		replyData[0] = 0x30;
////		packFloat(increment, replyData + 1);
////		CAN_Tx(replyData, 8, DEVICE_MASTER);
//
//		enc_before = raw_angle;
//	}

	E1A = HAL_GPIO_ReadPin(E1_A_GPIO_Port, E1_A_Pin);
	E1B = HAL_GPIO_ReadPin(E1_B_GPIO_Port, E1_B_Pin);
	curE1 = (E1A << 1) | E1B;

	E2A = HAL_GPIO_ReadPin(E2_A_GPIO_Port, E2_A_Pin);
	E2B = HAL_GPIO_ReadPin(E2_B_GPIO_Port, E2_B_Pin);
	curE2 = (E2A << 1) | E2B;

	if(curE1 != lastE1) {
		switch((lastE1 << 2) | curE1) {
		  case 0b0010:
		  case 0b1011:
		  case 0b1101:
		  case 0b0100:
			DCL.countE--;
			DCL.error = -(DCL.counts_to_go - DCL.countE);

			if (DCL.is_running && !DCL.dir) {
				DCL.speedE = speedTim(0);
				speedLE = __HAL_TIM_GET_COUNTER(&htim2);

				if ((DCL.counts_going) / 4 - 5 <= DCL.counts_going - DCL.error
						&& DCL.counts_going - DCL.error < (DCL.counts_going) / 4 + 5) {
					DCL.sum += DCL.speedE;
				} else if (DCL.counts_going - DCL.error == (DCL.counts_going) / 4 + 5) {
					spd = DCL.sum / 10;
					spdFlag = 1;
				}
			}

			break;

		  case 0b0001:
		  case 0b0111:
		  case 0b1110:
		  case 0b1000:
			DCL.countE++;
			DCL.error = (DCL.counts_to_go - DCL.countE);

//			DCL.speedE = __HAL_TIM_GET_COUNTER(&htim2);
//			__HAL_TIM_SET_COUNTER(&htim2, 0);

			if (DCL.is_running && DCL.dir) {
				DCL.speedE = speedTim(0);
				speedLE = __HAL_TIM_GET_COUNTER(&htim2);

				if ((DCL.counts_going) / 4 - 5 <= DCL.counts_going - DCL.error
						&& DCL.counts_going - DCL.error < (DCL.counts_going) / 4 + 5) {
					DCL.sum += DCL.speedE;
				} else if (DCL.counts_going - DCL.error == (DCL.counts_going) / 4 + 5) {
					spd = DCL.sum / 10;
					spdFlag = 1;
				}

				if (DCL.speedE <= 160 && accelCnt < 5) accelCnt++;
				if (accelCnt == 5) {
					accel = DCL.counts_going - DCL.error;
					accelCnt++;
				}
			}

			break;
		} lastE1 = curE1;
	}

	if(curE2 != lastE2) {
		switch((lastE2 << 2) | curE2) {
		  case 0b0010:
		  case 0b1011:
		  case 0b1101:
		  case 0b0100:
			DCR.countE++;
			DCR.error = (DCR.counts_to_go - DCR.countE);

			if (DCR.is_running && DCR.dir) {
				DCR.speedE = speedTim(1);
				speedRE = __HAL_TIM_GET_COUNTER(&htim2);

				if ((DCR.counts_going) / 4 - 5 <= DCR.counts_going - DCR.error
						&& DCR.counts_going - DCR.error < (DCR.counts_going) / 4 + 5) {
					DCR.sum += DCR.speedE;
				} else if (DCR.counts_going - DCR.error == (DCR.counts_going) / 4 + 5) {
					spd = DCR.sum / 10;
					spdFlag = 1;
				}
			}

			break;

		  case 0b0001:
		  case 0b0111:
		  case 0b1110:
		  case 0b1000:
			DCR.countE--;
			DCR.error = -(DCR.counts_to_go - DCR.countE);

			if (DCR.is_running && !DCR.dir) {
				DCR.speedE = speedTim(1);
				speedRE = __HAL_TIM_GET_COUNTER(&htim2);

				if ((DCR.counts_going) / 4 - 5 <= DCR.counts_going - DCR.error
						&& DCR.counts_going - DCR.error < (DCR.counts_going) / 4 + 5) {
					DCR.sum += DCR.speedE;
				} else if (DCR.counts_going - DCR.error == (DCR.counts_going) / 4 + 5) {
					spd = DCR.sum / 10;
					spdFlag = 1;
				}
			}

			break;
		} lastE2 = curE2;
	}



	if (DCL.stopped == -1 && speedTim(0) > 20000) {
		DCL.stopped = 1;

		if (dcl_program) dc_placed(0);
		if (swap == SWAP) dc_orient(target_ori);
//		send_int16(DCL.counts_going - DCL.error, replyData + 2);
	}

	if (DCR.stopped == -1 && speedTim(1) > 20000) {
		DCR.stopped = 1;

		if (dcr_program) dc_placed(1);
		if (swap == SWAP) dc_orient(target_ori);
	}

	if (phase != HOMING_DONE && DCL.stopped && DCR.stopped && !dc_flag) {
		start = HAL_GetTick();
		dc_flag = 1;
	} if (dc_flag && HAL_GetTick() - start > 100) {
		dc_flag = 0;
		dc_home();
	}

  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim -> Instance == stepper.htim_stepper -> Instance) Stepper_TIM_Interrupt();
    else if(htim -> Instance == DCL.htim_dc -> Instance) DC_TIM_Interrupt(&DCL);
    else if(htim -> Instance == DCR.htim_dc -> Instance) DC_TIM_Interrupt(&DCR);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 32-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  htim2.Init.Prescaler = 40-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  htim3.Init.Prescaler = 4-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
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
  htim4.Init.Prescaler = 4-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Motor1_1_Pin|Motor1_2_Pin|Motor2_1_Pin|Motor2_2_Pin
                          |CAN_SPEED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEP_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Motor1_1_Pin Motor1_2_Pin Motor2_1_Pin Motor2_2_Pin
                           CAN_SPEED_Pin */
  GPIO_InitStruct.Pin = Motor1_1_Pin|Motor1_2_Pin|Motor2_1_Pin|Motor2_2_Pin
                          |CAN_SPEED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_Pin DIR_Pin */
  GPIO_InitStruct.Pin = STEP_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : E2_B_Pin E2_A_Pin */
  GPIO_InitStruct.Pin = E2_B_Pin|E2_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : E1_B_Pin E1_A_Pin */
  GPIO_InitStruct.Pin = E1_B_Pin|E1_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
