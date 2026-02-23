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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "mpu6050.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define DT 0.01f

float accel[3], gyro[3], roll[3];
float dGyro, angle;
int16_t accelBias[3], gyroBias[3];
volatile uint8_t dataReady = 0;
int16_t encoderValue;
int16_t pwmValue, throttle, lqrOutput;
uint8_t estop = 0;
float eAccum = 0;

int __io_putchar(int ch){
	if(ch=='\n'){
		__io_putchar('\r');
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return 1;
}

int16_t control(float setAngle){
	float Kp = 5.0f;
	float Ki = 15.0f;
	float Kd = 0.5f;

	float e = setAngle - angle;
	eAccum = eAccum + e * DT;

	//Anty-windup
	if(eAccum > 22.0f){eAccum = 22.0f;}
	if(eAccum < 0.0f){eAccum = 0.0f;}
	if (estop == 1){eAccum = 0.0f;}
	if (encoderValue == 0){eAccum = 0.0f;}

	float P = Kp * e;
	float I = Ki * eAccum;
	float D = -1 * Kd * dGyro;

	float u = P+I+D;
	return (int16_t)u;
}

int16_t lqr(){
	float k1 = 31.6227766;
	float k2 = 31.36357997;
	float u1 = -k1 * ((angle * 0.01745329f) - 1.570796f);
	float u2 = -k2 * (dGyro * 0.01745329f);
	float u = u1 + u2 + 238.0f;
	return (int16_t)u;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim6){
		//
		// Readig data from MPU6050
		//

		mpu6050_ReadRoll(roll, &dGyro, accel, gyro);
		angle = roll[2]+90.f;
		//mpu6050_ReadRawBias(accelBias, gyroBias); // For reading biases

		//
		// Reading encoder value
		//

		encoderValue = __HAL_TIM_GET_COUNTER(&htim2);
		if(encoderValue < 0){encoderValue = 0;}
		if(encoderValue > 1000){encoderValue = 1000;}

		//
		// Safety stop
		//

		// Estop reset
		if((HAL_GPIO_ReadPin(button_GPIO_Port, button_Pin) == GPIO_PIN_RESET) && encoderValue == 0 && angle < 30){estop=0;}

		// Estop
		if(angle > 170.0f){estop=1;}

		//
		// Regulators - choose PID or LQR
		//

		if(encoderValue > 170){encoderValue = 170;}
		float setAngle = (float)encoderValue;

		// PID

		pwmValue = 160 + control(setAngle);

		// LQR

		//pwmValue = 160 + lqr();
		//lqrOutput = pwmValue;

		//
		// Motor manual controll
		//

		//pwmValue = 160 + encoderValue; - for manual motor control

		//
		// BLDC Motor controll
		//

		if(pwmValue < 160){pwmValue = 160;}
		if(pwmValue > 500){pwmValue = 500;}

		if((estop == 0) && encoderValue > 0){throttle = 1000+pwmValue;}
		else{throttle = 1000;}

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, throttle);

		//
		// Setting flag to main (for sending data via UART)
		//

		dataReady = 1;
	}
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Encoder Init
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // PWM Output Init
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);

  // MPU6050 Init
  HAL_Delay(1000);
  mpu6050_Init();
  HAL_Delay(1000);

  // Callback Timer
  HAL_TIM_Base_Start_IT(&htim6);

  // Setting Ready LED
  HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);

  while (1)
  {
	//
	// Telemetry via UART
	//
	if(dataReady==1){
		printf(">GyroX:%f\n", gyro[0]);
		printf(">GyroY:%f\n", gyro[1]);
		printf(">GyroZ:%f\n", gyro[2]);

		printf(">AccelX:%f\n", accel[0]);
		printf(">AccelY:%f\n", accel[1]);
		printf(">AccelZ:%f\n", accel[2]);

		printf(">RollAcc:%f\n", roll[0]);
		printf(">RollGyro:%f\n", roll[1]);
		printf(">RollFused:%f\n", roll[2]);

		printf(">Angle:%f\n", angle);
		printf(">Encoder:%d\n", encoderValue);
		printf(">Estop:%d\n", estop);
		printf(">Pwm:%d\n", pwmValue);
		printf(">throttle:%d\n", throttle);
		printf(">eAccum:%f\n", eAccum);
		printf(">lqrOutput:%d\n", lqrOutput);

		dataReady = 0;
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
#ifdef USE_FULL_ASSERT
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
