/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>

void MEMS_MPU6050_Init (void);
void MEMS_MPU6050_Read_Accel (void);
void MEMS_MPU6050_Read_Gyro (void );
void calculate_IMU_error(void);

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MEMS_MPU6050_ADDR 		0xD0
#define WHO_AM_I_REG 		0x75
#define PWR_MGMT_1_REG 		0x6B
#define SMPLRT_DIV_REG		0x19
#define CONFIG_REG_26		0x1A
#define GYRO_CONFIG_REG		0x1B
#define ACCEL_CONFIG_REG	0x1C

#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0X3D
#define ACCEL_YOUT_L		0x3E
#define ACCEL_ZOUT_H		0x3F
#define ACCEL_ZOUT_L		0x40

#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48

#define TRUE                0x1
#define FALSE               0x0


#define Stable_Speed   300
#define Full_Speed   700
#define Half_Speed   350
#define Test_Speed   600


int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float GyroErrorX = 0;
float GyroErrorY = 0;
float GyroErrorZ = 0;


float AccErrorX = 0;
float AccErrorY = 0;
float AccErrorZ = 0;

float Ax, Ay, Az, Gx, Gy, Gz;
float AngleRoll, AnglePitch;

unsigned char Rx_data[2];
unsigned char rec_buffer[10];
unsigned char start_buf = FALSE;
int BLE_pnt = 0;
int vHalf_Speed = 0;

int duty_cyle1 = Half_Speed;
int duty_cyle2 = Half_Speed;
int duty_cyle3 = Half_Speed;
int duty_cyle4 = Half_Speed;


int self_drive = 10;


char INC_MOTOR1 = FALSE;
char INC_MOTOR2 = FALSE;
char INC_MOTOR3 = FALSE;
char INC_MOTOR4 = FALSE;
char DRONE_STOP = FALSE;
char DRONE_UP   = FALSE;
char DRONE_IDLE = FALSE;
char DRONE_HOVER = FALSE;
char Stablize = FALSE;
char STAB1 = FALSE;
char STAB2 = FALSE;
char CALIBRATE = FALSE;


float xG = 0;
float yG = 0;
float zG = 0;
int rotate = 0;

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
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500); // Add a small delay
  MEMS_MPU6050_Init();

  calculate_IMU_error();

  htim3.Instance->CCR1 = duty_cyle1;
  htim3.Instance->CCR2 = duty_cyle2;
  htim3.Instance->CCR3 = duty_cyle3;
  htim3.Instance->CCR4 = duty_cyle4;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_UART_Receive_IT(&huart1, Rx_data, 1);
  //unsigned char response[125]; //Enough to return all holding-r's
  HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_RESET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_RESET);

	  HAL_Delay(5);
	  MEMS_MPU6050_Read_Accel ();
	  HAL_Delay(5);
	  MEMS_MPU6050_Read_Gyro();
	  HAL_Delay(5);

	  if (DRONE_HOVER)
	  {
		  DRONE_HOVER = FALSE;
	  }

	  if (DRONE_IDLE)
	  {
		  duty_cyle1 = 10;
		  duty_cyle2 = 10;
		  duty_cyle3 = 10;
		  duty_cyle4 = 10;
		  htim3.Instance->CCR1 = duty_cyle1;
		  htim3.Instance->CCR2 = duty_cyle2;
		  htim3.Instance->CCR3 = duty_cyle3;
		  htim3.Instance->CCR4 = duty_cyle4;
		  DRONE_IDLE = FALSE;
	  }

	  if (INC_MOTOR1)
	  {
		  htim3.Instance->CCR1 = Half_Speed;
		  htim3.Instance->CCR2 = Half_Speed;
		  HAL_Delay(2000);
		  htim3.Instance->CCR1 = duty_cyle1;
		  htim3.Instance->CCR2 = duty_cyle2;
		  INC_MOTOR1 = FALSE;
	  }

	  if (INC_MOTOR2)
	  {
		  htim3.Instance->CCR3 = Half_Speed;
		  htim3.Instance->CCR4 = Half_Speed;
		  HAL_Delay(2000);
		  htim3.Instance->CCR3 = duty_cyle3;
		  htim3.Instance->CCR4 = duty_cyle4;
		  INC_MOTOR2 = FALSE;
	  }

	  if (INC_MOTOR3)
	  {
		  duty_cyle3 += 20;
		  if (duty_cyle3 > Full_Speed)
			  duty_cyle3 = Full_Speed; // full speed
		  INC_MOTOR3 = FALSE;
		  htim3.Instance->CCR3 = duty_cyle3;
	  }

	  if (INC_MOTOR4)
	  {
		  duty_cyle4 += 20;
		  if (duty_cyle4 > Full_Speed)
			  duty_cyle4 = Full_Speed; // full speed
		  INC_MOTOR4 = FALSE;
		  htim3.Instance->CCR4 = duty_cyle4;
	  }

	  if (DRONE_UP)
	  {
		  duty_cyle1 += 20;
		  duty_cyle2 += 20;
		  duty_cyle3 += 20;
		  duty_cyle4 += 20;
		  if ((duty_cyle1 > Full_Speed) || (duty_cyle2 > Full_Speed) || (duty_cyle3 > Full_Speed) || (duty_cyle4 > Full_Speed))
		  {
			  duty_cyle1 -= 20;
			  duty_cyle2 -= 20;
			  duty_cyle3 -= 20;
			  duty_cyle4 -= 20;
			  Stablize = TRUE;
		  }
		  htim3.Instance->CCR1 = duty_cyle1;
		  htim3.Instance->CCR2 = duty_cyle2;
		  htim3.Instance->CCR3 = duty_cyle3;
		  htim3.Instance->CCR4 = duty_cyle4;
		  DRONE_UP = FALSE;
	  }

	  if (DRONE_STOP)
	  {
		  // Smoothly decrement each motor down to 0 instead of instant stop
		  int step = 20; // decrement step per iteration
		  while ((duty_cyle1 > 0) || (duty_cyle2 > 0) || (duty_cyle3 > 0) || (duty_cyle4 > 0))
		  {
			  if (duty_cyle1 > 0)
			  {
				  duty_cyle1 -= step;
				  if (duty_cyle1 < 0) duty_cyle1 = 0;
			  }
			  if (duty_cyle2 > 0)
			  {
				  duty_cyle2 -= step;
				  if (duty_cyle2 < 0) duty_cyle2 = 0;
			  }
			  if (duty_cyle3 > 0)
			  {
				  duty_cyle3 -= step;
				  if (duty_cyle3 < 0) duty_cyle3 = 0;
			  }
			  if (duty_cyle4 > 0)
			  {
				  duty_cyle4 -= step;
				  if (duty_cyle4 < 0) duty_cyle4 = 0;
			  }

			  htim3.Instance->CCR1 = duty_cyle1;
			  htim3.Instance->CCR2 = duty_cyle2;
			  htim3.Instance->CCR3 = duty_cyle3;
			  htim3.Instance->CCR4 = duty_cyle4;
			  HAL_Delay(20);
		  }
		  DRONE_STOP = FALSE;
	  }

	  // Set constant PWM duty cycles (no ramping)
	  htim3.Instance->CCR1 = duty_cyle1;
	  htim3.Instance->CCR2 = duty_cyle2;
	  htim3.Instance->CCR3 = duty_cyle3;
	  htim3.Instance->CCR4 = duty_cyle4;

	  HAL_Delay(20);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 800;
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
  sConfigOC.Pulse = 400;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void MEMS_MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MEMS_MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MEMS_MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);
		HAL_Delay(100); // Add a small delay
		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		// Set Sample Rate Divider, Accelerometer, and Gyroscope configurations
		HAL_I2C_Mem_Write(&hi2c1, MEMS_MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
		Data = 0x00;
		HAL_Delay(100); // Add a small delay
		HAL_I2C_Mem_Write(&hi2c1, MEMS_MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
		Data = 0x00;
		HAL_Delay(100); // Add a small delay
		HAL_I2C_Mem_Write(&hi2c1, MEMS_MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
		HAL_Delay(100); // Add a small delay
	}

}

void MEMS_MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MEMS_MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;

	if (Ay > 0.4) // positive value
	{
		HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_SET);
		HAL_Delay(50);
		STAB2 = TRUE;
	}
	if (Ay < -0.4)
	{
		HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_SET);
		HAL_Delay(50);
		STAB1 = TRUE;
	}
}

void ClearUARTFlags( void)
{
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_PE);
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_FE);
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_NE);
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
}

void calculate_IMU_error()
{
	int c = 0;
	uint8_t Rec_Data[6];

	AccErrorX = 0;
	AccErrorY = 0;
	AccErrorZ = 0;

	while (c < 200) {
		HAL_I2C_Mem_Read (&hi2c1, MEMS_MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, 1000);

		Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
		Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
		Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

		AccErrorX += Accel_X_RAW/16384.0;
		AccErrorY += Accel_Y_RAW/16384.0;
		AccErrorZ += Accel_Z_RAW/16384.0;
		c++;
  }
  //Divide the sum by 200 to get the error value

	AccErrorX = AccErrorX / 200;
	AccErrorY = AccErrorY / 200;
	AccErrorZ = AccErrorZ / 200;

	c = 0;
  // Read gyro values 200 times
	GyroErrorX = 0;
    GyroErrorY = 0;
    GyroErrorZ = 0;

	while (c < 200) {

		HAL_I2C_Mem_Read (&hi2c1, MEMS_MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, 1000);

		Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
		Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
		Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
	    // Sum all readings
		GyroErrorX = GyroErrorX + (Gyro_X_RAW / 131.0);
		GyroErrorY = GyroErrorY + (Gyro_Y_RAW / 131.0);
		GyroErrorZ = GyroErrorZ + (Gyro_Z_RAW / 131.0);
	    c++;
	}
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}

void MEMS_MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];


	HAL_I2C_Mem_Read (&hi2c1, MEMS_MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;


	Gx = Gx - GyroErrorX; // GyroErrorX ~(-0.56)
	Gy = Gy - GyroErrorY; // GyroErrorY ~(2)
	Gz = Gz - GyroErrorZ; // GyroErrorZ ~ (-0.8)

	if (Gx > 1)
	{
		HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_SET);
		STAB1 = TRUE;
		HAL_Delay(500);
	}

	if (Gx < -1)
	{
		HAL_GPIO_WritePin(GPIOC, LED2_Pin, GPIO_PIN_SET);
		STAB2 = TRUE;
		HAL_Delay(500);
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		if ((Rx_data[0] == 'M') ||
				(Rx_data[0] == '0') ||
				(Rx_data[0] == '1') ||
				(Rx_data[0] == '2') ||
				(Rx_data[0] == '3') ||
				(Rx_data[0] == 'X') ||
				(Rx_data[0] == 'U') ||
				(Rx_data[0] == 'H') ||
				(Rx_data[0] == 'S'))
		{
			rec_buffer[BLE_pnt] = Rx_data[0];
			BLE_pnt++;
			if (BLE_pnt >= 2)
			{
				if ((rec_buffer[0] == 'M') && (rec_buffer[1] == 'H'))
				{
					DRONE_HOVER = TRUE;
				} else
				if ((rec_buffer[0] == 'M') && (rec_buffer[1] == 'S'))
				{
					DRONE_IDLE = TRUE;
				} else
				if ((rec_buffer[0] == 'M') && (rec_buffer[1] == '0'))
				{
					INC_MOTOR1 = TRUE;
				}
				else if ((rec_buffer[0] == 'M') && (rec_buffer[1] == '1'))
				{
					INC_MOTOR2 = TRUE;
				}
				else if ((rec_buffer[0] == 'M') && (rec_buffer[1] == '2'))
				{
					INC_MOTOR3 = TRUE;
				}
				else if ((rec_buffer[0] == 'M') && (rec_buffer[1] == '3'))
				{
					INC_MOTOR4 = TRUE;
				}
				else if((rec_buffer[0] == 'M') && (rec_buffer[1] == 'U'))
				{
					DRONE_UP = TRUE;
				}
				else if((rec_buffer[0] == 'M') && (rec_buffer[1] == 'X'))
				{
					DRONE_STOP = TRUE;
				}
				else
				{
					INC_MOTOR1 = FALSE;
					INC_MOTOR2 = FALSE;
					INC_MOTOR3 = FALSE;
					INC_MOTOR4 = FALSE;
					DRONE_UP = FALSE;
				}
				BLE_pnt = 0;
			}
		} else
			BLE_pnt = 0;

		HAL_UART_Receive_IT(&huart1, Rx_data, 1);
/*		rec_buffer[index] = Rx_data[0];
		index++;
		if (index > 10)
			index = 9;
		HAL_UART_Receive_IT(&huart1, Rx_data, 2);
		*/
//		if (index >= 8)
//			CompletePacket = 1;
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
