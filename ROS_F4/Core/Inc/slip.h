/*
 * slip.h
 *
 *  Created on: Mar 31, 2025
 *      Author: hzy
 */

#ifndef INC_SLIP_H_
#define INC_SLIP_H_

#include <stdint.h>
#include <math.h>

// PID参数结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float Error;        // 当前误差
    float PrevError;    // 上一次误差
    float PrevPrevError;// 上上次误差
    float OutMax;       // 输出最大值
    float OutMin;       // 输出最小值
} PID_IncTypeDef;

void PID_IncInit(PID_IncTypeDef *pid, float kp, float ki, float kd, float outMax, float outMin);
float PID_IncCalculate(PID_IncTypeDef *pid, float target, float actual);

#endif /* INC_SLIP_H_ */
