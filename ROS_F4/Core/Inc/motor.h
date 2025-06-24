/*
 * motor.h
 *
 *  Created on: Feb 26, 2025
 *      Author: hzy
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"

void Motor_Left(int ch);
void Motor_Right(int ch);
void set_pwm(int ch,int speed);
uint32_t Read_Encoder_Count(TIM_HandleTypeDef *htim);
float Calculate_Encoder_Speed(int32_t delta);

#endif /* INC_MOTOR_H_ */
