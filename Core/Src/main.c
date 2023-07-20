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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Store incoming values from user input with USART
uint8_t byte;
uint8_t byte2[2];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define INLET 0
#define ZONE1 1
#define ZONE2 2
#define ZONE3 3
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/**
 * The following definitions are global variables used throughout the controlling
 * of the reservoir system.
 */

// Text buffer used to format and send strings with USART
uint8_t txd_msg_buffer[256] = {0};

// Flag to control servo motor location validation
int atLocation = 0;

// Flags to control RUN MODE
int goToRUNMODE = 0;
int currZoneIndex = -1;

// Percentage to display to Timer Board dual 7-Segment display
int DigitPct = 99;

// Clock Definitions
int clock_secs = 0;
int rpm_secs = 0;
int seconds = 0;
int mins = 0;
int hours = 0;
int clock_flag = 0;

// Distance Sensor Definitions
uint8_t us100_buffer[2];
uint16_t distance;
uint8_t us100_Rx_flag;
uint8_t cmd_dist = 0x55;

// RPM Definitions
int rpm_tick_count = 0;
int DCMotorRPM = 0;

// Servo Motor Definitions
int TIM3_DCVAL =  40;
int TIM3_CH1_STEP = 10;
int Motor_Step_Delay = 50;
int TIM2_Ch1_DCVAL = 500;
int TIM2_CH1_STEP = 20;


// Zone Struct
/**
 * Struct containing all zone-specific information required to run simulation.
 *
 * @param motorRPMSetting RPM value from {0, 1, ..., 9} defining the
 * relative PWM input by user
 * @param runTimeSetting relative scaled time from {01, 02, ..., 24} defining
 * relative run-time of pump
 * @param realTime real time scale from runTimeSetting
 * @param zoneTimeElapsed a continuously updated value represented the amount
 * of time water has been pumped to the zone
 * @param zoneName
 * @param complete bool describing if we have spent the full time required at
 * the zone
 */
struct Zones {
	unsigned int motorRPMSetting;
	unsigned int runTimeSetting;
	unsigned int realTime;
//	unsigned int waterLevel;
//	unsigned int waterCapacity;
	unsigned int zoneTimeElapsed;
	char* zoneName;
	bool complete;
};

// Array storing the 4 zones, {INLET, ZONE1, ZONE2, ZONE3}
struct Zones zoneArray[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);



/* USER CODE BEGIN PFP */

static void DIGIT_A_Display(uint8_t DIGIT_A);
static void DIGIT_B_Display(uint8_t DIGIT_B);
static void DISPLAY_TIME();
static void ADC_Select_CH(int CH);
static void selectZone();
static void selectZoneOptions(unsigned int zoneNo);
int readUSERB1();
static void reservoirLevel();
int changeZone(int zone);
static void ledDisplay(int zone);
int getLocation(int zone);
static void initializeZones();
int getPWM(unsigned int currZone);
int manualControl();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Flag for Distance Sensor
uint8_t rcv_intpt_flag = 0;

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
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */


  	// Servo Motor Init
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	TIM2->PSC = 16-1;
	TIM2->ARR = 20000-1;
	TIM2->CCR1 = TIM2_Ch1_DCVAL;

	// Brushed DC Motor Init
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	TIM3->PSC = 16 - 1;
	TIM3->ARR = 10000-1;
	TIM3->CCR1 = 0;
	TIM3->CCR3 = 0;


	// Timer Init
	HAL_TIM_Base_Start_IT(&htim5);

	while (!goToRUNMODE)
	{
		selectZone();
		HAL_Delay(2000);
		if (readUSERB1())
		{
		  goToRUNMODE = 1;
		}
	}

	sprintf((char*)txd_msg_buffer, "\r\nRUN MODE\r\n");
	HAL_UART_Transmit(&huart6,txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	// Initialize the zoneArray with the given parameters
	initializeZones();

  /* USER CODE END 2 */


  /* Infinite loop */

  /* USER CODE BEGIN WHILE */
  atLocation = 0;
  while (1)
  {
	  if (atLocation)
	  {
		  if (currZoneIndex == INLET)
		  {
			  if (zoneArray[INLET].zoneTimeElapsed < zoneArray[INLET].realTime)
			  {
				  TIM3->CCR3 = 0;
				  TIM3->CCR1 = getPWM(zoneArray[INLET].motorRPMSetting);
			  }
			  else
			  {
				  TIM3->CCR3 = 0;
				  TIM3->CCR1 = 0;
			      zoneArray[INLET].complete = true;
				  atLocation = false;
			  }
		  }
		  else if (currZoneIndex == ZONE1 || currZoneIndex == ZONE2 || currZoneIndex == ZONE3)
		  {
			  if (zoneArray[currZoneIndex].zoneTimeElapsed < zoneArray[currZoneIndex].realTime)
				  {
					  TIM3->CCR3 = getPWM(zoneArray[currZoneIndex].motorRPMSetting);
					  TIM3->CCR1 = 0;
				  }
				  else
				  {
					  TIM3->CCR3 = 0;
					  TIM3->CCR1 = 0;
				      zoneArray[currZoneIndex].complete = true;
					  atLocation = false;
				  }
		  }
		  reservoirLevel();
		  sprintf((char*)txd_msg_buffer, "\r\n-- %s -- PWM : %d -- RPM: %d -- Depth: %d\r\n",
				  zoneArray[currZoneIndex].zoneName, getPWM(zoneArray[currZoneIndex].motorRPMSetting),
				  DCMotorRPM, DigitPct);
		  HAL_UART_Transmit(&huart6,txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	  }
	  else
	  {
		  int index = 0;
		  bool zoneFound = false;

		  while(index < 4 && !zoneFound)
		  {
			  if (!zoneArray[index].complete)
			  {
				  zoneFound	= true;
				  currZoneIndex = index;
			  }
			  index++;
		  }
		  if (zoneArray[ZONE3].complete)
		  {
			  sprintf((char*)txd_msg_buffer, "\r\nAll Zones Complete!\r\n");
			  HAL_UART_Transmit(&huart6,txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
			  return 0;
		  }

		  while (!changeZone(currZoneIndex)){}
		  ledDisplay(currZoneIndex);
	      HAL_Delay(100);
		  atLocation = 1;
	  }

	  // Check if the reservoir is empty
	  if (atLocation && !DigitPct && currZoneIndex != INLET)
	  {
		  TIM3->CCR3 = 0;
		  TIM3->CCR1 = 0;
	      HAL_Delay(500);
		  atLocation = 0;
		  zoneArray[INLET].complete = false;
		  zoneArray[INLET].zoneTimeElapsed = 0;
	  }

  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_9;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 500-1;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 160-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 10-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, BCD_A3_Pin|BCD_A2_Pin|BCD_A1_Pin|BCD_A0_Pin
                          |BCD_B1_Pin|BCD_B2_Pin|BCD_B3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|BLU_Pin|GRN_Pin|RED_Pin
                          |BCD_B0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BCD_A3_Pin BCD_A2_Pin BCD_A1_Pin BCD_A0_Pin
                           BCD_B1_Pin BCD_B2_Pin BCD_B3_Pin */
  GPIO_InitStruct.Pin = BCD_A3_Pin|BCD_A2_Pin|BCD_A1_Pin|BCD_A0_Pin
                          |BCD_B1_Pin|BCD_B2_Pin|BCD_B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin BLU_Pin GRN_Pin RED_Pin
                           BCD_B0_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|BLU_Pin|GRN_Pin|RED_Pin
                          |BCD_B0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_Tick_Pin */
  GPIO_InitStruct.Pin = RPM_Tick_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_Tick_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/**
 * Bi-Directional USART Callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART6)
	{
		HAL_UART_Transmit(&huart6, &byte, 1, 100);
		rcv_intpt_flag = 1;
	}
	if (huart->Instance == USART1){
		us100_Rx_flag = 1;
	}
}

/**
 * Display digit to the most significant (left) 7-Segment display on Timer PCB
 *
 * @param DIGIT_A digit to display
 */
void DIGIT_A_Display(uint8_t DIGIT_A)
{
	 switch(DIGIT_A)
	 {
	 case 0:
		 HAL_GPIO_WritePin(GPIOC, BCD_A0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 1:
		 HAL_GPIO_WritePin(GPIOC, BCD_A0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 2:
		 HAL_GPIO_WritePin(GPIOC, BCD_A0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 3:
		 HAL_GPIO_WritePin(GPIOC, BCD_A0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 4:
		 HAL_GPIO_WritePin(GPIOC, BCD_A0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 5:
		 HAL_GPIO_WritePin(GPIOC, BCD_A0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 6:
		 HAL_GPIO_WritePin(GPIOC, BCD_A0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 7:
		 HAL_GPIO_WritePin(GPIOC, BCD_A0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A3_Pin, GPIO_PIN_RESET);
	 break;
	 case 8:
		 HAL_GPIO_WritePin(GPIOC, BCD_A0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A3_Pin, GPIO_PIN_SET);
	 break;
	 case 9:
		 HAL_GPIO_WritePin(GPIOC, BCD_A0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_A3_Pin, GPIO_PIN_SET);
	 break;
	 }
}

/**
 * Display digit to the least significant (left) 7-Segment display on Timer PCB
 *
 * @param DIGIT_B digit to display
 */
void DIGIT_B_Display(uint8_t DIGIT_B)
{
	 switch(DIGIT_B)
	 {
	 case 0:
		 HAL_GPIO_WritePin(GPIOA, BCD_B0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 1:
		 HAL_GPIO_WritePin(GPIOA, BCD_B0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 2:
		 HAL_GPIO_WritePin(GPIOA, BCD_B0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 3:
		 HAL_GPIO_WritePin(GPIOA, BCD_B0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 4:
		 HAL_GPIO_WritePin(GPIOA, BCD_B0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 5:
		 HAL_GPIO_WritePin(GPIOA, BCD_B0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 6:
		 HAL_GPIO_WritePin(GPIOA, BCD_B0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 7:
		 HAL_GPIO_WritePin(GPIOA, BCD_B0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B2_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B3_Pin, GPIO_PIN_RESET);
	 break;
	 case 8:
		 HAL_GPIO_WritePin(GPIOA, BCD_B0_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B3_Pin, GPIO_PIN_SET);
	 break;
	 case 9:
		 HAL_GPIO_WritePin(GPIOA, BCD_B0_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOC, BCD_B3_Pin, GPIO_PIN_SET);
	 break;
	 }
}

/**
 * Select channel for ADC/potentiometer use.
 *
 * @param CH integer value represent channel
 *
 * @warning PLEASE BE SURE THAT YOU DON'T SELECT AN INACCESSABLE CHANNEL DUE
 * TO A GPIO PIN BEING USED FOR A PURPOSE OTHER THAN FOR AN ADC CHANNEL!
 */
void ADC_Select_CH(int CH)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	switch(CH)
	{
		case 0:
		sConfig.Channel = ADC_CHANNEL_0;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 1:
		sConfig.Channel = ADC_CHANNEL_1;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 2:
		sConfig.Channel = ADC_CHANNEL_2;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 3:
		sConfig.Channel = ADC_CHANNEL_3;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 4:
		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 5:
		sConfig.Channel = ADC_CHANNEL_5;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 6:
		sConfig.Channel = ADC_CHANNEL_6;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 7:
		sConfig.Channel = ADC_CHANNEL_7;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 8:
		sConfig.Channel = ADC_CHANNEL_8;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 9:
		sConfig.Channel = ADC_CHANNEL_9;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 10:
		sConfig.Channel = ADC_CHANNEL_10;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 11:
		sConfig.Channel = ADC_CHANNEL_11;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 12:
		sConfig.Channel = ADC_CHANNEL_12;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 13:
		sConfig.Channel = ADC_CHANNEL_13;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 14:
		sConfig.Channel = ADC_CHANNEL_14;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 15:
		sConfig.Channel = ADC_CHANNEL_15;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	}
}

/**
 * Timer/Clock callback.
 *
 * Includes functionality for counting up RPM of DC Motor, and time elapsed for specific zones
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM5)
	{
		clock_secs += 1;
		clock_flag = 1;
		DCMotorRPM = ( (rpm_tick_count*60)/40);
		rpm_tick_count = 0;


		if (currZoneIndex == INLET)
		{
				zoneArray[INLET].zoneTimeElapsed += 1;
		}
		else if (currZoneIndex == ZONE1)
		{
				zoneArray[ZONE1].zoneTimeElapsed += 1;
		}
		else if (currZoneIndex == ZONE2)
		{
				zoneArray[ZONE2].zoneTimeElapsed += 1;
		}
		else if (currZoneIndex == ZONE3)
		{
				zoneArray[ZONE3].zoneTimeElapsed += 1;
		}

	}
}

/**
 * Display time to terminal using USART, formatted in hours : mins : seconds.
 */
static void DISPLAY_TIME()
{
	int temp_clock_secs = clock_secs;
	hours = temp_clock_secs/3600;
	mins = (temp_clock_secs%3600)/60;
	seconds = (temp_clock_secs-(hours*3600)-(mins*60));
	sprintf((char*)txd_msg_buffer, "\r\nCLOCK TIME (REAL)\r\n HOURS  MINS   SEC  \r\n  %-2u  :  %-2u  :  %-2u  ", hours, mins, seconds);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
}


/**
 * RPM Sensor callback. Counts up rpm_tick_count every time the speed sensor detects a slot.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == RPM_Tick_Pin)
	{
		rpm_tick_count += 1;
	}
}


/**
 * Prompt user to select a zone to configure, then call subroutine to find and load parameters.
 */
void selectZone()
{
	sprintf((char*)txd_msg_buffer, "\r\nSETUP MODE"
								   "\r\nSelect Zone:"
								   "\r\n0) Inlet"
								   "\r\n1) Zone 1"
								   "\r\n2) Zone 2"
								   "\r\n3) Zone 3\r\n");
	HAL_UART_Transmit(&huart6,txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	rcv_intpt_flag = 00;
	HAL_UART_Receive_IT(&huart6, &byte, 1);

	while (rcv_intpt_flag == (00));

	rcv_intpt_flag = 00;
	unsigned int zoneNo = byte-48;

	sprintf((char*)txd_msg_buffer, "\r\n\r\nZone: %u", zoneNo);
	HAL_UART_Transmit(&huart6,txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	selectZoneOptions(zoneNo);
}

/**
 * Prompt user to input parameters for RPM and designated time for a given zone zoneNo
 *
 * @param zoneNo zone to update values.
 */
void selectZoneOptions(unsigned zoneNo)
{
	  // Print Zone Motor RPM Options
	  sprintf((char*)txd_msg_buffer, "\r\n\r\nZone %u Motor RPM Options:"
									 "\r\n0) Manual Control"
									 "\r\n1) 10 PWM"
			  	  	  	  	  	  	 "\r\n2) 20 PWM"
									 "\r\n3) 30 PWM"
									 "\r\n4) 40 PWM"
			  	  	  	  	  	  	 "\r\n5) 50 PWM"
									 "\r\n6) 60 PWM"
									 "\r\n7) 70 PWM"
									 "\r\n8) 85 PWM"
									 "\r\n9) 99 PWM\r\n", zoneNo);
	  HAL_UART_Transmit(&huart6,txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	  rcv_intpt_flag = 00;
	  HAL_UART_Receive_IT(&huart6, &byte, 1);
	  while (rcv_intpt_flag == (00));


	  unsigned int motorRPMSetting = byte-'0';
	  zoneArray[zoneNo].motorRPMSetting = motorRPMSetting;



	  sprintf((char*)txd_msg_buffer, "\r\n\r\nMotor RPM Setting: %u");
	  	  	  	  	  	  	  	  	 "\r\n\r\nZone %u Run-Time Options (Scaled Time):"
			  	  	  	  	  	  	 "\r\nEnter a number between 01 and 24 (1 : 0.1 mins, 24 : 2.4 mins)\r\n", zoneArray[zoneNo].motorRPMSetting);
	  HAL_UART_Transmit(&huart6,txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	  rcv_intpt_flag = 00;
	  HAL_UART_Receive_IT(&huart6, &byte2, 2);
	  while (rcv_intpt_flag == (00));

	  unsigned int runTimeSetting = ((byte2[0] - '0')*10) + (byte2[1]-'0');
	  zoneArray[zoneNo].runTimeSetting = runTimeSetting;

	  sprintf((char*)txd_msg_buffer, "\r\n\r\nRun-Time Setting: %u\r\n", zoneArray[zoneNo].runTimeSetting);
	  HAL_UART_Transmit(&huart6,txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
}

/**
 * Read USERB1 input.
 */
int readUSERB1()
{
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
 * Update the reservoir level on the timer board 7-Segment Displays.
 */
void reservoirLevel()
{
	  int flagCount = 0;
	  us100_Rx_flag = 0;
	  HAL_UART_Receive_IT(&huart1, us100_buffer, 2);
	  HAL_UART_Transmit(&huart1, &cmd_dist, 1, 500);

	  while ((us100_Rx_flag == (00)) && flagCount < 1000 )
	  {
		  flagCount++;
	  }

	  distance = (us100_buffer[0] << 8) + us100_buffer[1];
	  rcv_intpt_flag = 00;

	  if(clock_flag){
		  clock_flag = 0;

		  if (distance < 495 && distance > 50)
		  {
			  DigitPct = 100 - distance/5;
		  }
		  else if (distance > 544)
		  {
			  DigitPct = 0;
		  }
		  else if (distance < 50)
		  {
			  DigitPct = 99;
		  }
		  uint8_t DIGIT_A = DigitPct/10;
		  uint8_t DIGIT_B = DigitPct%10;
		  DIGIT_B_Display(DIGIT_B);
		  DIGIT_A_Display(DIGIT_A);
	  }
	  HAL_Delay(50);
}

/**
 * Change the Servo Motor angle to the angle corresponding to the zone.
 *
 * @param zone integer referring to zone
 */
int changeZone(int zone)
{
	TIM2->CCR1 = getLocation(zone);
	atLocation = 1;
	return 1;
}

/**
 * Fetch the angle location for the servo motor for a given zone.
 *
 * @param zone given zone to fetch location
 * @return int value of location for servo motor
 */
int getLocation(int zone)
{
	if (zone == INLET)
	{
		return 500;
	}
	else if (zone == ZONE1)
	{
		return 1250;
	}
	else if (zone == ZONE2)
	{
		return 1750;
	}
	else if (zone == ZONE3)
	{
		return 2250;
	}
	else
	{
		return 0;
	}
}

/**
 * Display the corresponding LED colour for a given zone.
 *
 * @param zone given zone to display LED
 */
void ledDisplay(int zone)
{
	HAL_GPIO_WritePin(GPIOA, LD2_Pin|BLU_Pin|GRN_Pin|RED_Pin, GPIO_PIN_RESET);

	if (zone == INLET)
	{
		  HAL_GPIO_WritePin(GPIOA, BLU_Pin|RED_Pin, GPIO_PIN_SET);
	}
	else if ( zone == ZONE1)
	{
		  HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_SET);
	}
	else if (zone == ZONE2)
	{
		  HAL_GPIO_WritePin(GPIOA, GRN_Pin, GPIO_PIN_SET);
	}
	else if (zone == ZONE3)
	{
		  HAL_GPIO_WritePin(GPIOA, BLU_Pin, GPIO_PIN_SET);
	}
	else
	{
		// Do Nothing
	}
}

/**
 * Fetch the corresponding PWM based on the user motor PWM/RPM setting.
 *
 * @param motorRPMSetting value between 0 and 9 (inclusive) representing PWM
 * @return PWM value for DC Brushed Motor
 */
int getPWM(unsigned int motorRPMSetting )
{
	switch(motorRPMSetting)
	{
	case(0):
		return manualControl();
	case(8):
		return (0.85)*(TIM3->ARR);
	case(9):
		return (0.99)*(TIM3->ARR);
	default:
		return (0.1*motorRPMSetting)*(TIM3->ARR);
	}
}

/**
 * Initialize zone parameters.
 */
void initializeZones()
{
	// INLET Zone
	zoneArray[INLET].complete = false;
	zoneArray[INLET].realTime = (zoneArray[0].runTimeSetting*60)/10;
	zoneArray[INLET].zoneTimeElapsed = 0;
	zoneArray[INLET].zoneName = "Inlet";
	// Zone 1
	zoneArray[ZONE1].complete = false;
	zoneArray[ZONE1].realTime = (zoneArray[1].runTimeSetting*60)/10;
	zoneArray[ZONE1].zoneTimeElapsed = 0;
	zoneArray[ZONE1].zoneName = "Zone 1";

	// Zone 2
	zoneArray[ZONE2].complete = false;
	zoneArray[ZONE2].realTime = (zoneArray[2].runTimeSetting*60)/10;
	zoneArray[ZONE2].zoneTimeElapsed = 0;
	zoneArray[ZONE2].zoneName = "Zone 2";


	// Zone 3
//	zoneArray[ZONE3].waterLevel = 0;
//	zoneArray[ZONE3].waterCapacity = 14000;
	zoneArray[ZONE3].complete = false;
	zoneArray[ZONE3].realTime = (zoneArray[3].runTimeSetting*60)/10;
	zoneArray[ZONE3].zoneTimeElapsed = 0;
	zoneArray[ZONE3].zoneName = "Zone 3";
}

/**
 * Fetch ADC/Potentiometer read and convert to percentage of PWM to control DC Brushed Motor.
 *
 * @return percentage of PWM to control DC Brushed Motor
 */
int manualControl()
{
	ADC_Select_CH(9);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	uint8_t ADC_CH9 = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	return (( (ADC_CH9 << 6) + (ADC_CH9 << 5) + (ADC_CH9 << 2)) >> 8 )*10000/100;
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
