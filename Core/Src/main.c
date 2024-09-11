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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi 3.1415
#define p2r pi/1000
#define PULSE_PER_ROUND 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int32_t PosCnt,Cnttmp,speed;
int16_t CountValue=0,RealVel,DesiredSpeed, pwm;
uint16_t AngPos1, AngPos0,CntVel;
uint8_t PreviousState,Speedmode,tick_tim4=0, tick_tim5=0, dir;
bool run = false;
float CurPos=0,DesiredPos,CurVel, DesiredVel, DesiredLength, CurLength;	
char Rx_indx, Rx_Buffer[20],Rx_data[2];

// PID parameter //
const float Kp = 19.811;
const float Ki = 0;
const float Kb = 0;
const float Kd = 0.64627;
const float alpha = 0.1; //He so loc khau D
const float Ts = 0.005;
const int16_t HILIM = 20; 
const int16_t LOLIM = 0;	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
int16_t PID_Controller(float desired_value, float current_value);
void Run_Motor(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*----------------------------------- UART1 -----------------------------------*/
struct __FILE
{
	int handle;
	/* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};

int fputc(int ch, FILE*f)
{
	char tempch = ch;
	
	HAL_UART_Transmit(&huart1, (uint8_t*)&tempch, 1, 100);
	return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t i;
	if(huart->Instance == USART1) //uart1
		{
			if(Rx_indx==0)
				for (i=0;i<20;i++) Rx_Buffer[i] = 0;
			
			switch(Rx_data[0])
			{
				/* dung dong co */
				case 'e':
					run = false;
					break;
            
        /* dong co chay */
        case 'r':
          run = true;
          break;
        case 'b':
  				//reset();
					break;
        case 's':
          //DesiredPos = atoi(Rx_Buffer);
					DesiredLength = atoi(Rx_Buffer);
					DesiredLength = -DesiredLength;
					DesiredPos = DesiredLength*2*pi/10; //DesiredLength*2*pi/10;
          memset(Rx_Buffer, 0, sizeof(Rx_Buffer));
          Rx_indx = 0;
          break;
        case 'v':
          DesiredSpeed = atoi(Rx_Buffer);
					DesiredVel = DesiredSpeed * pi/30; //rad/s
          memset(Rx_Buffer, 0, sizeof(Rx_Buffer));
          Rx_indx = 0;
          break;    
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '.':
        case '-':
					Rx_Buffer[Rx_indx++] |= Rx_data[0];
					break; 
				default:
					break;
			}
			HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_data,1);
		}
}
/*------------------------------- KET THUC UART1 -------------------------------*/
void HAL_GPIO_EXTI_Callbaack(uint16_t GPIO_Pin){}
/*--------------------------- NGAT NGOAI HAI KENH A B --------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Channel_B_Pin)
	{
		unsigned char State0;
		State0 = (State0<<1) | HAL_GPIO_ReadPin(Channel_A_GPIO_Port, Channel_A_Pin);
		State0= (State0<<1) | HAL_GPIO_ReadPin(Channel_B_GPIO_Port, Channel_B_Pin);
		State0 = State0&0x03;
		switch (State0) {
			case 0:
				if(PreviousState==1) CountValue++;
				else CountValue--;
			break;
			case 1:
				if(PreviousState==3) CountValue++;
				else CountValue--;
			break;
			case 2:
				if(PreviousState==0) CountValue++;
				else CountValue--;
			break;
			case 3:
				if(PreviousState==2) CountValue++;
				else CountValue--;
			break;
			}
		PreviousState = State0;
		CntVel++;
		if (CountValue>=2000) {
			CountValue = 0;
			PosCnt++;
		}
		else if	(CountValue<=-2000) {
			CountValue = 0;
			PosCnt--;
		}
	}
	if (GPIO_Pin == Channel_A_Pin)
	{
		unsigned char State1;
		State1 = (State1<<1) | HAL_GPIO_ReadPin(Channel_A_GPIO_Port, Channel_A_Pin);
		State1 = (State1<<1) | HAL_GPIO_ReadPin(Channel_B_GPIO_Port, Channel_B_Pin);
		State1 = State1&0x03;
		switch (State1) {
			case 0:
				if(PreviousState==1) CountValue++;
				else CountValue--;
			break;
			case 1:
				if(PreviousState==3) CountValue++;
				else CountValue--;
			break;
			case 2:
				if(PreviousState==0) CountValue++;
				else CountValue--;
			break;
			case 3:
				if(PreviousState==2) CountValue++;
				else CountValue--;
			break;
			}
		PreviousState = State1;
		CntVel++;
		if (CountValue>=2000) {
			CountValue = 0;
			PosCnt++;
		}
		else if	(CountValue<=-2000) {
			CountValue = 0;
			PosCnt--;
		}
	}
}
/*---------------------- KET THUC NGAT NGOAI HAI KENH A B ----------------------*/

/*----------------------- TIMER4 - VAN TOC, VI TRI, PD CONTROLLER ------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
	if (htim->Instance == TIM4) //Interrupt 5ms
	{
			CurPos = PosCnt*2*pi+CountValue*p2r;	// Position calculation
			CurLength = 10*CurPos/(2*pi); //10*CurPos/(2*pi);
			Cnttmp = CntVel;
			CntVel = 0;
			RealVel = (Cnttmp*60)/(4*PULSE_PER_ROUND*Ts); //RPM
			CurVel = (Cnttmp*2*pi)/(4*PULSE_PER_ROUND*Ts); //rad/s
			
			if (run == true) {
//				HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, GPIO_PIN_SET);			
//				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwm);
				pwm = PID_Controller(DesiredPos, CurPos);
				Run_Motor();
			}
			else 
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
	}
	if (htim->Instance == TIM5) //Interrupt 10ms
	{
			tick_tim5++;
			if ((run == 1)&&(tick_tim5 == 5))
			{
				tick_tim5 = 0;
				//printf("V%d\r \n",RealVel);
				//printf("P%f\r \n",CurPos);
				printf("P%f\r \n",-CurLength);
			}
	}
}

void Run_Motor(void)
{
	if (dir == 1) //Chieu thuan
	{
		HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, GPIO_PIN_SET);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwm);
	}
	else if (dir == 2) //Chieu nghich
	{
		HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwm);
	}
	else if (dir == 0) //Dung
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
}

int16_t PID_Controller(float desired_value, float current_value)
{
	static float e_reset;
	static float u_i_pre;
	static float u_df_pre;
	static float ek_pre;
	
	float ek;
	float u_p, u_i, u_k, u_d, u_df;
	int16_t u_out;
	
	//Tinh sai so
	ek = desired_value - current_value;
	
	if (ek > 0)
	{
		dir = 1; //Chieu thuan
	}
	else if (ek < 0)
	{
		dir = 2; //Chieu nghich
		ek = -ek;
	}
	else
		dir = 0; //Stop
	
	//Khau P
	u_p = Kp*ek;
	
	//Khau I
	u_i = u_i_pre + Ki*Ts*ek + Kb*Ts*e_reset;
	u_i_pre = u_i;
	
	//Khau D
	u_d = Kd*(ek-ek_pre)/Ts;
	ek_pre = ek;
	u_df = (1-alpha)*u_df_pre + alpha*u_d;
	u_df_pre = u_df;
	
	//PID Controller
	u_k = u_p + u_i + u_df;
	
	if (u_k > HILIM) {
		u_out = HILIM;
		e_reset = u_out - u_k;
	}
	else if (u_k < LOLIM) {
		u_out = LOLIM;
		e_reset = u_out - u_k;
	}
	else {
		u_out = (int16_t)u_k;
		e_reset = 0;
	}
	
	return u_out;
}

/*------------------- KET THUC TIMER4 - VAN TOC, VI TRI, PD CONTROLLER -------------------*/




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
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim4); //Timer 4 interrupt 5ms
	HAL_TIM_Base_Start_IT(&htim5); //Timer 5 interrupt 10ms
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //Timer tao pwm
	HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_data,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 11;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 23999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4;
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
  htim5.Init.Prescaler = 23999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 9;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Dir_Pin */
  GPIO_InitStruct.Pin = Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Dir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Channel_A_Pin Channel_B_Pin */
  GPIO_InitStruct.Pin = Channel_A_Pin|Channel_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
