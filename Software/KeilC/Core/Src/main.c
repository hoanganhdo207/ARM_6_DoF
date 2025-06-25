/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h> 
#include <stdlib.h> 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMER_INTERRUPT 3000
#define STEP_PER_EVOLUTION 6400
#define ANGLE_PER_STEP 0.05625	
//#define RATIO_1 8.33
//#define RATIO_1 4.8125
#define RATIO_1 (8.33*5.18)
#define RATIO_2 6.4166
#define RATIO_3 (5.0833*13.7335) 
#define RATIO_4 1
#define RATIO_5 3.75
#define RATIO_6 1 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//------------------------CONTROL---------------------
typedef struct {
	GPIO_TypeDef* DIR_Port ; 
	uint16_t DIR_Pin ;
	GPIO_TypeDef* Pulse_Port ; 
	uint16_t Pulse_Pin ;
	float ratio ; 
}	StepperMotor ; 

StepperMotor motor1 = {DIR_1_GPIO_Port, DIR_1_Pin , PULSE_1_GPIO_Port , PULSE_1_Pin , (float)RATIO_1} ;
StepperMotor motor2 =	{DIR_2_GPIO_Port, DIR_2_Pin , PULSE_2_GPIO_Port , PULSE_2_Pin , (float)RATIO_2} ;
StepperMotor motor3 = {DIR_3_GPIO_Port, DIR_3_Pin , PULSE_3_GPIO_Port , PULSE_3_Pin , (float)RATIO_3} ;
StepperMotor motor4 = {DIR_4_GPIO_Port, DIR_4_Pin , PULSE_4_GPIO_Port , PULSE_4_Pin , (float)RATIO_4} ;
StepperMotor motor5 = {DIR_5_GPIO_Port, DIR_5_Pin , PULSE_5_GPIO_Port , PULSE_5_Pin , (float)RATIO_5} ;
StepperMotor motor6 = {DIR_6_GPIO_Port, DIR_6_Pin , PULSE_6_GPIO_Port , PULSE_6_Pin , (float)RATIO_6} ;

float trajectory[10000] ; 
uint16_t trajec_index = 0 ; 
uint8_t step_state = 0 ;
volatile uint32_t count = 0 ; 
uint32_t step_target ; 
uint8_t joint ; 
//------------------------UART------------------------
uint8_t rxData , data_flag = 1 ; 
char rxBuffer[10] ; 
uint8_t rxIndex = 0 ; 
char test[10] = "OK" ;  
long long time , pre_time ; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
float Angle_to_Step(float angle , int Ratio) ;
void Run_Stepper (float angle , StepperMotor motor , GPIO_PinState dir)  ; 
void Start(void) ; 
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart1 , &rxData , 1) ; 
	//HAL_TIM_Base_Start_IT(&htim1) ;
	Start() ; 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
		if(data_flag == 0) 
		{
				Run_Stepper(trajectory[trajec_index++] , motor1 , GPIO_PIN_SET) ; 
				Run_Stepper(trajectory[trajec_index++] , motor2 , GPIO_PIN_SET) ; 
				Run_Stepper(trajectory[trajec_index++] , motor3 , GPIO_PIN_SET) ; 
				Run_Stepper(trajectory[trajec_index++] , motor4 , GPIO_PIN_SET) ; 
				Run_Stepper(trajectory[trajec_index++] , motor5 , GPIO_PIN_SET) ; 
				Run_Stepper(trajectory[trajec_index++] , motor6 , GPIO_PIN_SET) ; 
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PULSE_1_Pin|DIR_1_Pin|PULSE_2_Pin|DIR_2_Pin
                          |PULSE_3_Pin|DIR_3_Pin|EN_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_6_Pin|PULSE_6_Pin|DIR_5_Pin|PULSE_5_Pin
                          |DIR_4_Pin|PULSE_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PULSE_1_Pin DIR_1_Pin PULSE_2_Pin DIR_2_Pin
                           PULSE_3_Pin DIR_3_Pin EN_1_Pin */
  GPIO_InitStruct.Pin = PULSE_1_Pin|DIR_1_Pin|PULSE_2_Pin|DIR_2_Pin
                          |PULSE_3_Pin|DIR_3_Pin|EN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Limit_SW_1_Pin Limit_SW_2_Pin */
  GPIO_InitStruct.Pin = Limit_SW_1_Pin|Limit_SW_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Limit_SW_3_Pin Limit_SW_5_Pin */
  GPIO_InitStruct.Pin = Limit_SW_3_Pin|Limit_SW_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_6_Pin PULSE_6_Pin DIR_5_Pin PULSE_5_Pin
                           DIR_4_Pin PULSE_4_Pin */
  GPIO_InitStruct.Pin = DIR_6_Pin|PULSE_6_Pin|DIR_5_Pin|PULSE_5_Pin
                          |DIR_4_Pin|PULSE_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1) 
	{  
		if(rxData != '\n') 
		{
			data_flag = 1 ; 
			rxBuffer[rxIndex++] = rxData ; 
		}
		else if(rxData == ' ') 
		{
			trajectory[trajec_index++] = atof(rxBuffer) ; 
			rxIndex = 0 ; 
			memset(rxBuffer , 0 , sizeof(rxBuffer)) ; 
		}
		else if(rxData == '\n') 
		{
			data_flag = 0 ;
			trajec_index = 0 ; 
		}
		HAL_UART_Receive_IT(&huart1 , &rxData , 1) ; 
	}
}

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(count < step_target) 
	{
		if(step_state == 0) //2 ISR for 1 PULSE HIGH->LOW
		{
			step_state = 1 ; 
			HAL_GPIO_WritePin(PULSE_5_GPIO_Port , PULSE_5_Pin , GPIO_PIN_SET) ;
		} 
		else 
		{
			step_state = 0 ; 
			HAL_GPIO_WritePin(PULSE_5_GPIO_Port , PULSE_5_Pin , GPIO_PIN_RESET) ;
			count++ ; 
		}
	}
	else 
	{
		step_target = 0 ; 
		count = 0 ; 
	}
}
*/

void Run_Stepper (float angle , StepperMotor motor , GPIO_PinState dir) 
{
	int step_target = (angle /(float)ANGLE_PER_STEP) * motor.ratio ;
	HAL_GPIO_WritePin(motor.DIR_Port , motor.DIR_Pin , dir) ;
	for(int i = 0 ; i<step_target ; i++) 
	{
		HAL_GPIO_WritePin(motor.Pulse_Port , motor.Pulse_Pin , GPIO_PIN_SET) ; 
		HAL_Delay(1) ; 
		HAL_GPIO_WritePin(motor.Pulse_Port , motor.Pulse_Pin , GPIO_PIN_RESET) ; 
	}
}

void Start() 
{
	float step = 0 ; 
	while(1)
	{
		int sw_1 = HAL_GPIO_ReadPin(Limit_SW_1_GPIO_Port , Limit_SW_1_Pin) ; 
		int sw_2 = HAL_GPIO_ReadPin(Limit_SW_2_GPIO_Port , Limit_SW_1_Pin) ; 
		int sw_3 = HAL_GPIO_ReadPin(Limit_SW_3_GPIO_Port , Limit_SW_1_Pin) ; 
		int sw_5 = HAL_GPIO_ReadPin(Limit_SW_5_GPIO_Port , Limit_SW_5_Pin) ;
		
		if(sw_1 == 0) Run_Stepper(0 , motor1 , GPIO_PIN_SET) ;
		else Run_Stepper(step , motor1 , GPIO_PIN_SET) ;
		if(sw_2 == 0) Run_Stepper(0 , motor2 , GPIO_PIN_SET) ;
		else Run_Stepper(step , motor2 , GPIO_PIN_SET) ;
		if(sw_3 == 0) Run_Stepper(0 , motor3 , GPIO_PIN_SET) ;
		else Run_Stepper(step , motor3 , GPIO_PIN_SET) ;
		if(sw_5 == 0) Run_Stepper(0 , motor5 , GPIO_PIN_SET) ;
		else Run_Stepper(step , motor5 , GPIO_PIN_SET) ;
		step++ ; 
		if(sw_1 == 0 && sw_2 == 0 && sw_3 == 0 && sw_5 == 0) break ; 
		HAL_Delay(100) ; 
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
