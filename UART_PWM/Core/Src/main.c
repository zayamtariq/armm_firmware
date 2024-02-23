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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "UARTBuffer.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* UART VARIABLES: */

// receive
uint8_t rx_indx;
uint8_t rx_data[2]; // how much data are we receiving at a time (2 bytes currently)
uint8_t rx_buffer[100];
uint8_t transfer_cplt; // flag that tells us when transfer is complete

// UART buffer
UartBuffer IK_Message_Buffer;

/* MOTOR VARIABLES: */

// for counting steps left for each motor
int motor1_steps = 0;
int motor2_steps = 0;
int motor3_steps = 0;
int motor4_steps = 0;
int motor5_steps = 0;
int motor6_steps = 0;

// for holding acceleration and decleration constants for each motor (calculated after parsing uart message)
/*
 * acceleration_constant = (total_motor_steps * 8) / 10
 * deceleration_constant = (total_motor_steps * 2) / 10
 */
uint16_t accel1, decel1;
uint16_t accel2, decel2;
uint16_t accel3, decel3;
uint16_t accel4, decel4;
uint16_t accel5, decel5;
uint16_t accel6, decel6;

// incrementer/decrementer for each motor based on frequency provided at uart command
uint16_t incrementer_decrementer1 = 0;
uint16_t incrementer_decrementer2 = 0;
uint16_t incrementer_decrementer3 = 0;
uint16_t incrementer_decrementer4 = 0;
uint16_t incrementer_decrementer5 = 0;
uint16_t incrementer_decrementer6 = 0;

// current frequencies in SPS
uint16_t current_freq1 = 0;
uint16_t current_freq2 = 0;
uint16_t current_freq3 = 0;
uint16_t current_freq4 = 0;
uint16_t current_freq5 = 0;
uint16_t current_freq6 = 0;

// total frequencies needed to meet in SPS
uint16_t total_freq1 = 0;
uint16_t total_freq2 = 0;
uint16_t total_freq3 = 0;
uint16_t total_freq4 = 0;
uint16_t total_freq5 = 0;
uint16_t total_freq6 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM11_Init(void);
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  // start UART:
  HAL_UART_Receive_IT(&huart2, rx_data, 1); // interrupt based UART receive

  // initialize UART buffer:
  initializeBuffer(&IK_Message_Buffer);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (motor1_steps == 0 && motor2_steps == 0 && motor3_steps == 0 && motor4_steps == 0 && motor5_steps == 0 && motor6_steps == 0) {
		  // now we can pop off of the UART buffer and re-instantiate all of our variables
		  UartMessage recent_command = removeFromBuffer(&IK_Message_Buffer, &huart2);
		  if (!(strcmp(recent_command.message, "\0"))) {
			  // stop timer 11
			  HAL_TIM_Base_Stop_IT(&htim11); // start again when motor command comes through
		  } // if there are no messages to pop, we'll continue polling
		  else {
			  char command[6];
			  int motorValue1, motorFrequency1, motorValue2, motorFrequency2, motorValue3, motorFrequency3, motorValue4, motorFrequency4, motorValue5, motorFrequency5, motorValue6, motorFrequency6;
			  /* command - MOTOR
			   * motorValuen - how many pulses are being sent to the motor
			   * motorFrequencyn - in units of [steps per second]
			   */

			  /*
			   * Direction Pins
			   * PA11 - motor 1
			   * PA10 - motor 2
			   * PA9  - motor 3
			   * PC9  - motor 4
			   * PC8  - motor 5
			   * PC7  - motor 6
			   *
			   * for PWM:
			   * PA8  - motor 1
			   * PA0  - motor 2
			   * PA6  - motor 3
			   * PB6  - motor 4
			   * PC6  - motor 5
			   * PB14 - motor 6
			   *
			   * for UART:
			   * PA2 = UART_TX
			   * PA3 = UART_RX
			   *
			   */

			  if (sscanf((char *)recent_command.message, "%s %d %d %d %d %d %d %d %d %d %d %d %d", command, &motorValue1, &motorFrequency1, &motorValue2, &motorFrequency2, &motorValue3, &motorFrequency3, &motorValue4, &motorFrequency4, &motorValue5, &motorFrequency5, &motorValue6, &motorFrequency6) == 13) {

				  if (!(strcmp(command, "MOTOR"))) { // move motor N pulses

					  // populate counter for the number of steps left
					  motor1_steps = motorValue1;
					  motor2_steps = motorValue2;
					  motor3_steps = motorValue3;
					  motor4_steps = motorValue4;
					  motor5_steps = motorValue5;
					  motor6_steps = motorValue6;

					  // populate total frequency trying to reach
					  total_freq1 = motorFrequency1;
					  total_freq2 = motorFrequency2;
					  total_freq3 = motorFrequency3;
					  total_freq4 = motorFrequency4;
					  total_freq5 = motorFrequency5;
					  total_freq6 = motorFrequency6;

					  // populate current frequency that timer will initially run at
					  current_freq1 = motorFrequency1 / 2;
					  current_freq2 = motorFrequency2 / 2;
					  current_freq3 = motorFrequency3 / 2;
					  current_freq4 = motorFrequency4 / 2;
					  current_freq5 = motorFrequency5 / 2;
					  current_freq6 = motorFrequency6 / 2;

					  // calculate the acceleration and deceleration constants here
					  accel1 = (motorValue1 * 8) / 10;
					  decel1 = (motorValue1 * 2) / 10;
					  accel2 = (motorValue2 * 8) / 10;
					  decel2 = (motorValue2 * 2) / 10;
					  accel3 = (motorValue3 * 8) / 10;
					  decel3 = (motorValue3 * 2) / 10;
					  accel4 = (motorValue4 * 8) / 10;
					  decel4 = (motorValue4 * 2) / 10;
					  accel5 = (motorValue5 * 8) / 10;
					  decel5 = (motorValue5 * 2) / 10;
					  accel6 = (motorValue6 * 8) / 10;
					  decel6 = (motorValue6 * 2) / 10;

					  // set direction pins
					  if (motorValue1 < 0) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // set direction pin low for negative
					  else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); // set direction pin high for positive

					  if (motorValue2 < 0) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
					  else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

					  if (motorValue3 < 0) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
					  else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

					  if (motorValue4 < 0) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
					  else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

					  if (motorValue5 < 0) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
					  else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

					  if (motorValue6 < 0) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
					  else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

					  // calculate incrementer for acceleration based GPIO:
					  incrementer_decrementer1 = ((((84000000 / total_freq1) - (84000000 / (total_freq1 / 2))) / 2) + (84000000 / total_freq1)) * (decel1 / 65535);
					  incrementer_decrementer2 = ((((84000000 / total_freq2) - (84000000 / (total_freq2 / 2))) / 2) + (84000000 / total_freq2)) * (decel2 / 65535);
					  incrementer_decrementer3 = ((((84000000 / total_freq3) - (84000000 / (total_freq3 / 2))) / 2) + (84000000 / total_freq3)) * (decel3 / 65535);
					  incrementer_decrementer4 = ((((84000000 / total_freq4) - (84000000 / (total_freq4 / 2))) / 2) + (84000000 / total_freq4)) * (decel4 / 65535);
					  incrementer_decrementer5 = ((((84000000 / total_freq5) - (84000000 / (total_freq5 / 2))) / 2) + (84000000 / total_freq5)) * (decel5 / 65535);
					  incrementer_decrementer6 = ((((84000000 / total_freq6) - (84000000 / (total_freq6 / 2))) / 2) + (84000000 / total_freq6)) * (decel6 / 65535);

					  // __HAL_TIM_SET_AUTORELOAD(__HANDLE__, __AUTORELOAD__)
					  // change period to whatever period oughta be
					  __HAL_TIM_SET_AUTORELOAD(&htim1, (84000000) / (motorFrequency1 / 2));
					  __HAL_TIM_SET_AUTORELOAD(&htim2, (84000000) / (motorFrequency2 / 2));
					  __HAL_TIM_SET_AUTORELOAD(&htim3, (84000000) / (motorFrequency3 / 2));
					  __HAL_TIM_SET_AUTORELOAD(&htim4, (84000000) / (motorFrequency4 / 2));
					  __HAL_TIM_SET_AUTORELOAD(&htim8, (84000000) / (motorFrequency5 / 2));
					  __HAL_TIM_SET_AUTORELOAD(&htim12, (84000000) / (motorFrequency6 / 2));


					  __disable_irq();

					  // start motors again (at frequency of motorFrequencyn / 2)
					  if (motor1_steps != 0) HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
					  if (motor2_steps != 0) HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
					  if (motor3_steps != 0) HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
					  if (motor4_steps != 0) HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
					  if (motor5_steps != 0) HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
					  if (motor6_steps != 0) HAL_TIM_PWM_Start_IT(&htim12, TIM_CHANNEL_1);

					  // start timer 11
					  HAL_TIM_Base_Start_IT(&htim11);

					  __enable_irq();

					  // just to see some signs of life
					  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				  }
			  }
		  }
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 100;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void accelerationTimerISR()
{
	// update frequency of pwm timers

	if (motor1_steps >= accel1) {
		current_freq1 += incrementer_decrementer1;
		HAL_UART_Transmit(&huart2, (uint8_t *)"Accelerating 1\n", 15, 100);
	} else if (motor1_steps <= decel1 && motor1_steps != 0) {
		current_freq1 -= incrementer_decrementer1;
		HAL_UART_Transmit(&huart2, (uint8_t *)"Decelerating 1\n", 15, 100);
	} else if (motor1_steps < accel1 && motor1_steps > decel1) {
		current_freq1 = total_freq1;
		HAL_UART_Transmit(&huart2, (uint8_t *)"Chilling 1\n", 11, 100);
	}

	if (motor2_steps >= accel2) {
		current_freq2 += incrementer_decrementer2;
	} else if (motor2_steps <= decel2 && motor2_steps != 0) {
		current_freq2 -= incrementer_decrementer2;
	} else if (motor2_steps < accel2 && motor2_steps > decel2) {
		current_freq2 = total_freq2;
	}

	if (motor3_steps >= accel3) {
		current_freq3 += incrementer_decrementer3;
	} else if (motor3_steps <= decel3 && motor3_steps != 0) {
		current_freq3 -= incrementer_decrementer3;
	} else if (motor3_steps < accel3 && motor3_steps > decel3) {
		current_freq3 = total_freq3;
	}

	if (motor4_steps >= accel4) {
		current_freq4 += incrementer_decrementer4;
	} else if (motor4_steps <= decel1 && motor4_steps != 0) {
		current_freq4 -= incrementer_decrementer4;
	} else if (motor4_steps < accel1 && motor4_steps > decel1) {
		current_freq4 = total_freq4;
	}

	if (motor5_steps >= accel5) {
		current_freq5 += incrementer_decrementer5;
	} else if (motor5_steps <= decel5 && motor5_steps != 0) {
		current_freq5 -= incrementer_decrementer5;
	} else if (motor5_steps < accel5 && motor5_steps > decel5) {
		current_freq5 = total_freq5;
	}

	if (motor6_steps >= accel6) {
		current_freq6 += incrementer_decrementer6;
	} else if (motor6_steps <= decel6 && motor6_steps != 0) {
		current_freq6 -= incrementer_decrementer6;
	} else if (motor6_steps < accel6 && motor6_steps > decel6) {
		current_freq6 = total_freq6;
	}

	// stop all timers
	HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(&htim12, TIM_CHANNEL_1);

	// update all our periods now
	__HAL_TIM_SET_AUTORELOAD(&htim1, (84000000) / current_freq1);
	__HAL_TIM_SET_AUTORELOAD(&htim2, (84000000) / current_freq2);
	__HAL_TIM_SET_AUTORELOAD(&htim3, (84000000) / current_freq3);
	__HAL_TIM_SET_AUTORELOAD(&htim4, (84000000) / current_freq4);
	__HAL_TIM_SET_AUTORELOAD(&htim8, (84000000) / current_freq5);
	__HAL_TIM_SET_AUTORELOAD(&htim12, (84000000) / current_freq6);

	// start timers with new periods
	if (motor1_steps > 0) HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	if (motor2_steps > 0) HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	if (motor3_steps > 0) HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	if (motor4_steps > 0) HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
	if (motor5_steps > 0) HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
	if (motor6_steps > 0) HAL_TIM_PWM_Start_IT(&htim12, TIM_CHANNEL_1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM11)
	{
		accelerationTimerISR();
	}
}


// we come to this function after a successful receive
// huart = the uart that was configured for this receive
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  __disable_irq();

  uint8_t i;
  if (huart->Instance == USART2) { // make sure we're working with our correct instance of UART (PA2-3 = USART2 TX/RX)

	  if (rx_indx == 0) {
		  for (i = 0; i < 100; i++) {
			  rx_buffer[i] = 0; // clear the buffer for a new message once the user clicked enter
		  }
	  }

	  if (rx_data[0] != 13) { // enter button = ascii 13
		  rx_buffer[rx_indx++] = rx_data[0]; // reading a byte at a time in our RX buffer
	  } else {

		  /* NOW NEED TO ADD THIS TO OUR UART BUFFER */
		  while (!addToBuffer(&IK_Message_Buffer, (const char *) rx_buffer, huart)) {
			  HAL_UART_Transmit(&huart2, (uint8_t *) "Buffer Full\n", 12, 100);
			  __enable_irq();
			  // if all pwm callbacks are done then we can reliably push a message off the uart buffer here
			  __disable_irq();
		  } // keep spinning until we can successfully add to the buffer

		  rx_indx = 0; // if user has clicked the enter button, compare our rx_buffer with desired string, then clear the buffer for next message
		  transfer_cplt = 1; // set flag
		  HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, 100);
	  }

	  HAL_UART_Receive_IT(&huart2, rx_data, 1); // initialize to interrupt based receive
	  HAL_UART_Transmit(&huart2, rx_data, strlen((char *)rx_data), 100); // initialize to transmit as we write new data to the rx_data byte(s)

  }

  __enable_irq();
}

// this function gets called every time that *some* PWM timer sends a pulse
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1) {
		// --global_motor_flag;
		if (motor1_steps != 0) --motor1_steps;
		// if (global motor flag == 0) {disable interrupt, motherfucker!};
		if (motor1_steps <= 0) {
			HAL_UART_Transmit(&huart2, (uint8_t *)"1\n", 2, 100);
			HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
		}
	}
	else if (htim->Instance == TIM2) {
		// --global_motor_flag;
		if (motor2_steps != 0) --motor2_steps;
		// if (global motor flag == 0) {disable interrupt, motherfucker!};
		if (motor2_steps <= 0) {
			HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
		}
	}
	else if (htim->Instance == TIM3)
    {
    	// --global_motor_flag;
		if (motor3_steps != 0) --motor3_steps;
		// if (global motor flag == 0) {disable interrupt, motherfucker!};
		if (motor3_steps <= 0) {
			HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
		}
    }
	else if (htim->Instance == TIM4) {
    	// --global_motor_flag;
		if (motor4_steps != 0) --motor4_steps;
		// if (global motor flag == 0) {disable interrupt, motherfucker!};
		if (motor4_steps <= 0) {
			HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
		}
	}
	else if (htim->Instance == TIM8) {
    	// --global_motor_flag;
		if (motor5_steps != 0) --motor5_steps;
		// if (global motor flag == 0) {disable interrupt, motherfucker!};
		if (motor5_steps <= 0) {
			HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);
		}
	}
	else if (htim->Instance == TIM12) {
    	// --global_motor_flag;
		if (motor6_steps != 0) --motor6_steps;
		// if (global motor flag == 0) {disable interrupt, motherfucker!};
		if (motor6_steps <= 0) {
			HAL_TIM_PWM_Stop_IT(&htim12, TIM_CHANNEL_1);
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
