/*
 * slip.c
 *
 *  Created on: Mar 31, 2025
 *      Author: hzy
 */

#include "slip.h"

extern int32_t EncodeTotal[4];

float TargetSpeed = 0.0f;   // 目标速度(RPM或编码器计数)
float ActualSpeed = 0.0f;   // 实际速度(RPM或编码器计数)
float PWM_Duty = 0.0f;		// PWM占空比(0-100)

// 编码器相关变量
int32_t EncoderCount = 0;
int32_t PrevEncoderCount = 0;
uint32_t LastEncoderTime = 0;

void ccc_v_w(void)
{

}

void PID_IncInit(PID_IncTypeDef *pid, float kp, float ki, float kd, float outMax, float outMin)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Error = 0.0f;
    pid->PrevError = 0.0f;
    pid->PrevPrevError = 0.0f;
    pid->OutMax = outMax;
    pid->OutMin = outMin;
}

float PID_IncCalculate(PID_IncTypeDef *pid, float target, float actual)
{
    float increment = 0.0f;

    // 计算当前误差
    pid->Error = target - actual;

    // 增量式PID公式: Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
    increment = pid->Kp * (pid->Error - pid->PrevError) +
                pid->Ki * pid->Error +
                pid->Kd * (pid->Error - 2 * pid->PrevError + pid->PrevPrevError);

    // 更新误差历史
    pid->PrevPrevError = pid->PrevError;
    pid->PrevError = pid->Error;

    // 限制输出增量
    if(increment > pid->OutMax) {
        increment = pid->OutMax;
    } else if(increment < pid->OutMin) {
        increment = pid->OutMin;
    }

    return increment;
}
