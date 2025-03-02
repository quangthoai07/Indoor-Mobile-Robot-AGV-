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
#include "mainpp.h"
#include "data_struct.h"
#include "imu_data_struct.h"
#include <math.h>
#include "stm32f4xx_hal.h"
#include "defines.h"
#include "tm_stm32_mpu6050.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float   temperature=0;
TM_MPU6050_t MPU6050_Sensor;
TM_MPU6050_t MPU6050_Data0;
char     data[120];


// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;
float         angle_z;

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

unsigned long get_last_time() {return last_read_time;}
float get_last_x_angle() {return last_x_angle;}
float get_last_y_angle() {return last_y_angle;}
float get_last_z_angle() {return last_z_angle;}
float get_last_gyro_x_angle() {return last_gyro_x_angle;}
float get_last_gyro_y_angle() {return last_gyro_y_angle;}
float get_last_gyro_z_angle() {return last_gyro_z_angle;}

float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;

 float gyro_angle_z=0;

 void get_mpudata_average() {

   int read_count = 20;
   // Discard the first reading (don't know if this is needed or
   // not, however, it won't hurt.)
   // Read and average the raw values
   for (int i = 0; i < read_count; i++) {
     TM_MPU6050_ReadAll(&MPU6050_Sensor);
     base_x_gyro += MPU6050_Sensor.Gyroscope_X;
     base_y_gyro += MPU6050_Sensor.Gyroscope_Y;
     base_z_gyro += MPU6050_Sensor.Gyroscope_Z;
     base_x_accel +=  MPU6050_Sensor.Accelerometer_X;
     base_y_accel +=  MPU6050_Sensor.Accelerometer_Y;
     base_z_accel +=  MPU6050_Sensor.Accelerometer_Z;
   }

     base_x_gyro /= read_count;
     base_y_gyro /= read_count;
     base_z_gyro /= read_count;
     base_x_accel /= read_count;
     base_y_accel /= read_count;
     base_z_accel /= read_count;
 }

// This global variable tells us how to scale gyroscope data
float    GYRO_FACTOR=32768.0/250.0;

unsigned long t_now=0,t_last=0;
float dt=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

//Right
float PPR1 = 998;//=920x4/ 1070;pulse per revolution
const float Ts = 0.05;
float encoder_cnt1=0, pre_enc1=0;
float delta_enc1 = 0;

float vR_motor=0;
float wR_motor=0;
float vR_set=0, vR_real=0;
float v_robot=0;
float w_robot=0;

float duty_cycle1=0, R_duty1=0;
float Error1=0, pre_Error1=0, pre_pre_Error1=0;
float Kp1=50;//0.9, 50
float Ki1=1300;//30, 850
float Kd1=0.025;//0.003
float P_part1=0, I_part1=0, D_part1=0;
float udk1=0, pwm_out1=0, pre_pwm_out1=0,state1 = 0;
const float wheelRadius= 0.072;//0.0725
const float wheelBase=0.46;
const float pi=3.14159265359;
float angle_y;
float angle_x;
float angle_z=0;
const float RADIANS_TO_DEGREES = 180/pi; //180/3.14159
//Left
float PPR2 = 998;//=920x4/ 1070;pulse per revolution

float encoder_cnt2=0, pre_enc2=0;
float delta_enc2 = 0;

float vL_motor=0;
float wL_motor=0;
float  vL_set=0, vL_real=0;

float setSpeed2=0;
float duty_cycle2=0, duty2=0;
float Error2=0, pre_Error2=0, pre_pre_Error2=0;
float Kp2=50;//0.9
float Ki2=1300;//30
float Kd2=0.025;//0.003
float P_part2=0, I_part2=0, D_part2=0;
float udk2=0, pwm_out2=0, pre_pwm_out2=0,state2 = 0;


struct vel_data vel_data_rx;
struct vel_data vel_data_tx;

float current_x, current_y, current_theta=0;
float pre_x=0, pre_y=0, pre_theta=0;
float delta_theta=0;
struct vel_data vel_data_rx;
struct vel_data vel_data_tx;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
   HAL_TIM_Base_Start_IT(&htim2);
   HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1|TIM_CHANNEL_2);
   HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1|TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

   setup();
   while(TM_MPU6050_Init(&MPU6050_Sensor, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_250s) != TM_MPU6050_Result_Ok)
        	 {

        	 }

           //GYRO_FACTOR = 131.0;
        	 //const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159


        	 get_mpudata_average();

        	 set_last_read_angle_data(0, 0, 0, 0, 0, 0, 0);



     //	HAL_Delay(5000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	  //	  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	  	  loop();

	  	  //HAL_Delay(200);

//	  	  		t_last = HAL_GetTick();
//	  	  		/* Read all data from sensor 1 */
//	  	  		TM_MPU6050_ReadAll(&MPU6050_Sensor);
//	  	  		t_now = HAL_GetTick();
//
//	  	  		// Remove offsets and scale gyro data
//	  	  		//        float gyro_x = (MPU6050_Data0.Gyroscope_X - base_x_gyro)/GYRO_FACTOR;
//	  	  		//        float gyro_y = (MPU6050_Data0.Gyroscope_Y - base_y_gyro)/GYRO_FACTOR;
//	  	  		//        float gyro_z = (MPU6050_Data0.Gyroscope_Z - base_z_gyro)/GYRO_FACTOR;
//	  	  		float gyro_x = (MPU6050_Sensor.Gyroscope_X) / GYRO_FACTOR;
//	  	  		float gyro_y = (MPU6050_Sensor.Gyroscope_Y) / GYRO_FACTOR;
//	  	  		float gyro_z = (MPU6050_Sensor.Gyroscope_Z) / GYRO_FACTOR;
//
//	  	  		float accel_x = MPU6050_Sensor.Accelerometer_X; // - base_x_accel;
//	  	  		float accel_y = MPU6050_Sensor.Accelerometer_Y; // - base_y_accel;
//	  	  		float accel_z = MPU6050_Sensor.Accelerometer_Z; // - base_z_accel;
//
//	  	  		float accel_angle_y = atan(
//	  	  				-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2)))
//	  	  				* RADIANS_TO_DEGREES;
//	  	  		float accel_angle_x = atan(
//	  	  				accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2)))
//	  	  				* RADIANS_TO_DEGREES;
//	  	  		float accel_angle_z = 0;  //Accelerometer doesn't give z-angle
//
//	  	  		// Compute the (filtered) gyro angles
//	  	  		dt = (t_now - get_last_time()) / 1000.0;
//
//	  	  		float gyro_angle_x = gyro_x * dt + get_last_x_angle();
//	  	  		float gyro_angle_y = gyro_y * dt + get_last_y_angle();
//
//	  	  		int gz_threshold = 2; // gyro z raw data fluctuation threshold value when gyro doesn't move. It is up to your mpu6050. It is just a personal approach.
//	  	  		if (gyro_z < gz_threshold && gyro_z > -gz_threshold) // When gyro stands ignore the gyro z small fluctuations to prevent z angle irregular increments
//	  	  			gyro_z = 0;
//
//	  	  		gyro_angle_z = gyro_z * dt + get_last_z_angle();
//
//	  	  		// Compute the drifting gyro angles
//	  	  		float unfiltered_gyro_angle_x = gyro_x * dt + get_last_gyro_x_angle();
//	  	  		float unfiltered_gyro_angle_y = gyro_y * dt + get_last_gyro_y_angle();
//	  	  		float unfiltered_gyro_angle_z = gyro_z * dt + get_last_gyro_z_angle();
//
//	  	  		// Apply the complementary filter to figure out the change in angle - choice of alpha is
//	  	  		/* estimated now.  Alpha depends on the sampling rate... */
//	  	  		const float alpha = 0.96;
//	  	  		 angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
//	  	  		 angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
//	  	  		 angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
//
//	  	  		/* Update the saved data with the latest values */
//	  	  		set_last_read_angle_data(t_now, angle_x, angle_y, angle_z,
//	  	  				unfiltered_gyro_angle_x, unfiltered_gyro_angle_y,
//	  	  				unfiltered_gyro_angle_z);
//
//	  	  		/* Send the angle values to serial buffer	*/
////	  	  		HAL_UART_Transmit(&huart2, (uint8_t*) data, sprintf(data,
////	  	  						"Angle values\n- X:%3.4f\n- Y:%3.4f\n- Z:%3.4f\nTemperature\n- %3.4f\n\n\n",
////	  	  						angle_x,
////	  	  						angle_y,
////	  	  						angle_z,
////	  	  						MPU6050_Sensor.Temperature),
////	  	  				1000);
//	  	  		imu_data_tx.x=angle_x;
//	  	  		imu_data_tx.y=angle_y;
//	  	  		imu_data_tx.z=angle_z;

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 839;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  huart6.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void PID_Calculate1(){
			//realSpeed=(float)Encoder_GetSpeed()/Ts/1080;


			 //delta_enc=Encoder_GetSpeed();




				encoder_cnt1=__HAL_TIM_GET_COUNTER(&htim1);
				delta_enc1=encoder_cnt1-pre_enc1;



					//p=(int32_t)TIM1->CNT;

				if(delta_enc1>32768){
					delta_enc1 -= 65536;
				}
				else if(delta_enc1<-32768){
					delta_enc1 += 65536;
				}

				 vL_real=((delta_enc1/PPR1)/Ts)*(2*pi*wheelRadius);
				 pre_enc1=encoder_cnt1;

				 Error1=vL_set-vL_real;
				 if(Kp1!=0||Ki1!=0||Kd1!=0){

					 P_part1 =Kp1*(Error1-pre_Error1 );
					// P_part2 =Kp2*(Error2-pre1_Error2 );

					 I_part1 = 0.5*Ki1*Ts*(Error1 + pre_Error1);
					// I_part2 = 0.5*Ki2*Ts*(Error2 + pre1_Error2);

					 D_part1 = Kd1/Ts*( Error1 - 2*pre_Error1+ pre_pre_Error1);
					 //D_part2 = Kd2/Ts*( Error2 - 2*pre1_Error2+ pre2_Error2);

					 udk1 = pre_pwm_out1 + P_part1 + I_part1 + D_part1 ;
					 //udk2 = pre_pwm_out2 + P_part2 + I_part2 + D_part2 ;

								 //HAL_Delay(0.5);
								 //pwm_out=udk;
								 pre_pre_Error1 = pre_Error1;
								 //pre2_Error2 = pre1_Error2;

								 pre_Error1 = Error1;
								// pre1_Error2 = Error2;

								 pre_pwm_out1 = udk1;
								// pre_pwm_out2 = udk2;
				 }
		}

void PID_Calculate2(){

				encoder_cnt2=__HAL_TIM_GET_COUNTER(&htim4);


				 delta_enc2=encoder_cnt2-pre_enc2;


					//p=(int32_t)TIM1->CNT;



				if(delta_enc2>32768){
					delta_enc2 -= 65536;
				}
				else if(delta_enc2<-32768){
					delta_enc2 += 65536;
				}

				vR_real=((delta_enc2/PPR2)/Ts)*(2*pi*wheelRadius);
				pre_enc2=encoder_cnt2;


				Error2=vR_set-vR_real;

			 if(Kp2!=0||Ki2!=0||Kd2!=0){


				 P_part2 =Kp2*(Error2-pre_Error2 );


				 I_part2 = 0.5*Ki2*Ts*(Error2 + pre_Error2);


				 D_part2 = Kd2/Ts*( Error2 - 2*pre_Error2+ pre_pre_Error2);


				 udk2 = pre_pwm_out2 + P_part2 + I_part2 + D_part2 ;

				 pre_pre_Error2 = pre_Error2;


				 pre_Error2 = Error2;


				 pre_pwm_out2 = udk2;
			 }

	}

	void PWM_Calculate1()
		{
			if(udk1>1000){
				udk1=50;

			}
			else if (udk1<-1000){
				udk1=-50;

			}


			if(udk1==0){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_10, GPIO_PIN_RESET);

			}
			else if(udk1>0){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_10, GPIO_PIN_SET);

				duty_cycle1=udk1;


			}

			else{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_10, GPIO_PIN_RESET);

				duty_cycle1=-udk1;
				//duty_cycle2=-udk2;
			}
	//		duty_cycle=(TIM3->ARR + 1)*duty_cycle/100;
	//		TIM3->CCR1 = (uint32_t)duty_cycle;
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,duty_cycle1);
	//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,duty_cycle2);
		}

	void PWM_Calculate2()
			{

				if(udk2>1000){

					udk2=50;
				}
				else if (udk2<-1000){

					udk2=-50;
				}


				if(udk2==0){

					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC,  GPIO_PIN_12, GPIO_PIN_RESET);
							}
				else if(udk2>0){

					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC,  GPIO_PIN_12, GPIO_PIN_SET);

					duty_cycle2=udk2;

						}
				else{

					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOC,  GPIO_PIN_12, GPIO_PIN_RESET);

					duty_cycle2=-udk2;
				}

				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,duty_cycle2);
			}



void read_imu(){

	t_last = HAL_GetTick();
				  		/* Read all data from sensor 1 */
				  	TM_MPU6050_ReadAll(&MPU6050_Sensor);
				      //TM_MPU6050_ReadAccelerometer(&MPU6050_Sensor);
				      //TM_MPU6050_ReadGyroscope(&MPU6050_Sensor);
				  		t_now = HAL_GetTick();

				  		// Remove offsets and scale gyro data
				          float gyro_x = (MPU6050_Sensor.Gyroscope_X - base_x_gyro)/GYRO_FACTOR;
				          float gyro_y = (MPU6050_Sensor.Gyroscope_Y - base_y_gyro)/GYRO_FACTOR;
				          float gyro_z = (MPU6050_Sensor.Gyroscope_Z - base_z_gyro)/GYRO_FACTOR;
			//	  		    float gyro_x = (MPU6050_Sensor.Gyroscope_X)/GYRO_FACTOR;
			//	          float gyro_y = (MPU6050_Sensor.Gyroscope_Y)/GYRO_FACTOR;
			//	          float gyro_z = (MPU6050_Sensor.Gyroscope_Z)/GYRO_FACTOR;

				          float accel_x = MPU6050_Sensor.Accelerometer_X  - base_x_accel;
				          float accel_y = MPU6050_Sensor.Accelerometer_Y  - base_y_accel;
				          float accel_z = MPU6050_Sensor.Accelerometer_Z  - base_z_accel;

				          float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
				          float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
				          float accel_angle_z = 0;  //Accelerometer doesn't give z-angle

				          // Compute the (filtered) gyro angles
				          dt =(t_now - get_last_time())/1000.0;

				          float gyro_angle_x = gyro_x*dt + get_last_x_angle();
				          float gyro_angle_y = gyro_y*dt + get_last_y_angle();

				  				int gz_threshold = 2; // gyro z raw data fluctuation threshold value when gyro doesn't move. It is up to your mpu6050. It is just a personal approach.
				  				if(gyro_z < gz_threshold && gyro_z > -gz_threshold) // When gyro stands ignore the gyro z small fluctuations to prevent z angle irregular increments
				  					 gyro_z = 0;

				             gyro_angle_z = gyro_z*dt + get_last_z_angle();

				          // Compute the drifting gyro angles
				          float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
				          float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
				          float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();

				          // Apply the complementary filter to figure out the change in angle - choice of alpha is
				          /* estimated now.  Alpha depends on the sampling rate... */
				          const float alpha = 0.96;
				          float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
				          float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
				          angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

				  		    /* Update the saved data with the latest values */
				          set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);



}
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
		if(htim->Instance==htim2.Instance){
			  //setSpeed= vel_data_rx.v;
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

//			vR_set=vel_data_rx.v + vel_data_rx.w*wheelBase/2;//dong hoc nguoc
//			vL_set=vel_data_rx.v - vel_data_rx.w*wheelBase/2;

			PID_Calculate1();
			PWM_Calculate1();
			PID_Calculate2();
			PWM_Calculate2();

//			vel_data_tx.v=(vR_real+vL_real)/2;  //dong hoc nguoc
//			vel_data_tx.w=(vR_real-vL_real)/wheelBase;
			read_imu();
			imu_data_tx.z=angle_z;




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
