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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include "jy901.h"
#include "motor.h"
#include <string.h>
#include "delay.h"
#include "slip.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t IMU_data[44]={0};
uint8_t uart_data[40]={0};
uint8_t to_lcd_Buffer[40];

int left_pwm = 0;
int right_pwm = 0;

uint8_t IMU_FPS=0;
uint8_t cnt=0;
uint8_t cntt=0;
uint8_t IMU_FPS_sum=0;

uint32_t left_count = 0;
uint32_t right_count = 0;

float left_speed = 0 , right_speed = 0;

int n0=0,n1=0,n2=0,n3=0;
float v1 = 0;

extern User_USART JY901_data;
int flag = 1;

PID_IncTypeDef MotorPID;  // 声明PID控制器结构体变量
extern float x_speed;
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
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
  return ch;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	User_USART_Init(&JY901_data);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,uart_data,40);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,IMU_data,44);

	ST7735_Init();

	ST7735_DrawString(0,0,"F4 & ROS",ST7735_WHITE,ST7735_BLACK,&Font_11x18);
	ST7735_DrawString(0,58,"N0:",ST7735_WHITE,ST7735_BLACK,&Font_7x10);
	ST7735_DrawString(80,58,"N1:",ST7735_WHITE,ST7735_BLACK,&Font_7x10);
	ST7735_DrawString(0,68,"N2:",ST7735_WHITE,ST7735_BLACK,&Font_7x10);
	ST7735_DrawString(80,68,"N3:",ST7735_WHITE,ST7735_BLACK,&Font_7x10);

	ST7735_DrawString(0,18,"X :",ST7735_WHITE,ST7735_BLACK,&Font_7x10);
	ST7735_DrawString(0,28,"Y :",ST7735_WHITE,ST7735_BLACK,&Font_7x10);
	ST7735_DrawString(0,38,"Z :",ST7735_WHITE,ST7735_BLACK,&Font_7x10);
	ST7735_DrawString(0,48,"IMU_FPS :",ST7735_WHITE,ST7735_BLACK,&Font_7x10);

	PID_IncInit(&MotorPID, 0.2f, 0.07f, 0, 2000.0f, -2000.0f);//kp ki kd max min

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  set_pwm(1,left_pwm);
	  set_pwm(2,right_pwm);

//	  set_pwm(1,200);
//	  set_pwm(2,200);
//	  send_raspi();
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint8_t RXD_Buffer[33];
    if(huart == &huart1)
    {
        memcpy(RXD_Buffer, uart_data, Size);
        RXD_Buffer[Size] = '\0'; // 补0结束符

        char rxd_buf_char[50] = {0}; // 新建一个char数组，防止类型冲突
        memcpy(rxd_buf_char, RXD_Buffer, Size); // 拷贝到char数组中
        rxd_buf_char[Size] = '\0'; // 再补一个结束符

        if (rxd_buf_char[0] == '[')
        {
            char *comma_ptr = strchr(rxd_buf_char, ',');
            char *right_bracket_ptr = strchr(rxd_buf_char, ']');

            if (comma_ptr != NULL && right_bracket_ptr != NULL && comma_ptr < right_bracket_ptr)
            {
                char left_str[10] = {0};
                char right_str[10] = {0};

                memcpy(left_str, rxd_buf_char + 1, comma_ptr - (rxd_buf_char + 1));
                left_str[comma_ptr - (rxd_buf_char + 1)] = '\0';

                memcpy(right_str, comma_ptr + 1, right_bracket_ptr - (comma_ptr + 1));
                right_str[right_bracket_ptr - (comma_ptr + 1)] = '\0';

                left_pwm = atoi(left_str);
                right_pwm = atoi(right_str);
            }
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_data, 40); // 继续接收
    }
    if(huart == &huart2)
    {
        IMU_FPS_sum++;
//        HAL_UART_Transmit(&huart1, "IMU_data get!!!!!!!\r\n", 22,0xff);
        memcpy(JY901_data.RxBuffer,IMU_data, JY901_data.Rx_len);
        JY901_data.Rx_len = sizeof(IMU_data);
        JY901_Process();
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2,IMU_data,44);
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim6))	//5hz,0.2s一次
    {
//    	n1++;
    	left_count = (uint32_t)Read_Encoder_Count(&htim2);
    	right_count = (uint32_t)Read_Encoder_Count(&htim3);

    	left_speed = Calculate_Encoder_Speed(left_count);
    	right_speed = Calculate_Encoder_Speed(right_count);
//		for(int i = 0 ; i < 4 ;i++)
//		{
//			angular_vel[i] = (EncodeTotal[i] / 0.2) * (2 * PI / (52 * 20));
//			linear_vel[i] = angular_vel[i] * (WHEEL_DIAMETER / 2);
//		}
//    	send_raspi();
    }

    if (htim == (&htim5))	//100Mhz,10ms一次
    {
    	//send_raspi();
		IMU_FPS=IMU_FPS_sum;
		IMU_FPS_sum=0;
		update_velocity();
		cnt++;
		if(cnt == 10)
		{
			cnt = 0;
			x_speed = 0;
		}

		if(n0 % 4 == 0)
		{
			ST7735_DrawInt(20,58,left_pwm,ST7735_WHITE,ST7735_BLACK,&Font_7x10);
			ST7735_DrawInt(100,58,right_pwm,ST7735_WHITE,ST7735_BLACK,&Font_7x10);
			ST7735_DrawInt(20,68,left_count,ST7735_WHITE,ST7735_BLACK,&Font_7x10);
			ST7735_DrawInt(100,68,right_count,ST7735_WHITE,ST7735_BLACK,&Font_7x10);

			ST7735_DrawFloat(30,18, JY901_data.angle.angle[0],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
			ST7735_DrawFloat(30,28, JY901_data.angle.angle[1],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
			ST7735_DrawFloat(30,38, JY901_data.angle.angle[2],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
			ST7735_DrawFloat(70,48,IMU_FPS,ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
		}
		else if(n0 %4 == 1)
		{
//			ST7735_DrawInt(20,58,linear_vel[0],ST7735_WHITE,ST7735_BLACK,&Font_7x10);
			ST7735_DrawFloat(20,58, n0,ST7735_WHITE,ST7735_BLACK,&Font_7x10,2);
			ST7735_DrawInt(100,58,n1,ST7735_WHITE,ST7735_BLACK,&Font_7x10);
			ST7735_DrawInt(20,68,n2,ST7735_WHITE,ST7735_BLACK,&Font_7x10);
			ST7735_DrawInt(100,68,n3,ST7735_WHITE,ST7735_BLACK,&Font_7x10);

			ST7735_DrawFloat(30,18, JY901_data.acc.a[0],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
			ST7735_DrawFloat(30,28, JY901_data.acc.a[1],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
			ST7735_DrawFloat(30,38, JY901_data.acc.a[2],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
			ST7735_DrawFloat(70,48,IMU_FPS,ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
		}
		else if(n0 %4 == 2)
		{
			ST7735_DrawInt(20,58,v1,ST7735_WHITE,ST7735_BLACK,&Font_7x10);

			ST7735_DrawFloat(30,18, JY901_data.w.w[0],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
			ST7735_DrawFloat(30,28, JY901_data.w.w[1],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
			ST7735_DrawFloat(30,38, JY901_data.w.w[2],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
			ST7735_DrawFloat(70,48,IMU_FPS,ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
		}
		else if(n0 %4 == 3)
		{
			ST7735_DrawInt(20,58,n0,ST7735_WHITE,ST7735_BLACK,&Font_7x10);
			ST7735_DrawInt(100,58,n1,ST7735_WHITE,ST7735_BLACK,&Font_7x10);
			ST7735_DrawInt(20,68,n2,ST7735_WHITE,ST7735_BLACK,&Font_7x10);
			ST7735_DrawInt(120,68,n3,ST7735_WHITE,ST7735_BLACK,&Font_7x10);

			ST7735_DrawFloat(30,18, JY901_data.q.q[0],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
			ST7735_DrawFloat(30,28, JY901_data.q.q[1],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
			ST7735_DrawFloat(30,38, JY901_data.q.q[2],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
			ST7735_DrawFloat(70,48,JY901_data.q.q[3],ST7735_WHITE,ST7735_BLACK,&Font_7x10,1);
		}
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin==KEY0_Pin)
    {
        n0++;
    }
    if(GPIO_Pin==KEY1_Pin)
    {
        n1++;
    }
    if(GPIO_Pin==KEY2_Pin)
    {
        n2++;
    }
    if(GPIO_Pin==KEY3_Pin)
    {
        n3++;
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
