/*
 * motor.c
 *
 *  Created on: Feb 26, 2025
 *      Author: hzy
 */
#include "motor.h"

#define SAMPLE_PERIOD_MS     200.0f
#define ENCODER_RESOLUTION   7100.0f
#define PI 3.1415926f
#define WHEEL_DIAMETER 0.075f // 车轮直径(米)

extern int n0;
//PE5\PE6 TIM9 CH1\2 右\左电机	AIN1_1 AIN2_1控制左侧电机正反转 BIN1_1 BIN2_1 控制右侧电机正反转
//PB3\PB5 TIM3 CJ1\2 编码器 采用输入捕获
//PD4\5 右前轮 1、0正转，0、1反转  PD6\7 右后轮 1、0正转，0、1反转
//PD2\3 左前轮 0、1正转，1、0反转  PD0\1 左后轮 0、1正转，1、0反转

void Motor_Left(int ch)//ch 0正 1反
{
	if(ch == 0)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	}
	else if(ch == 1)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
	}
}

void Motor_Right(int ch)//ch 0正 1反
{
	if(ch == 0)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);
	}
	else if(ch == 1)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_RESET);
	}
}

void set_pwm(int ch,int speed)		//speed 0-1000
{
	if(ch==1) //左轮
	{
		if(speed>=0)
		{
			Motor_Left(0);
			__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,speed);
		}
		else
		{
			Motor_Left(1);
			__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,-speed);
		}
	}
	else if(ch==2)  //右轮
	{
		if(speed>=0)
		{
			Motor_Right(0);
			__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,speed);
		}
		else
		{
			Motor_Right(1);
			__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,-speed);
		}
	}
}

uint32_t Read_Encoder_Count(TIM_HandleTypeDef *htim)
{
	uint32_t count = 0;
	count = __HAL_TIM_GET_COUNTER(htim);
	__HAL_TIM_SET_COUNTER(htim,0);
    return count;
}

float Calculate_Encoder_Speed(int32_t delta)
{
    const float wheel_circumference = PI * WHEEL_DIAMETER;

    // delta 就是本周期的编码器计数增量（脉冲数）
    // 转/秒 = 脉冲数 / 每圈脉冲数 / 采样周期（秒）
    float rps = (delta * (1000.0f / SAMPLE_PERIOD_MS)) / ENCODER_RESOLUTION;

    // 线速度 = 转速 × 圆周长
    return rps * wheel_circumference;
}


