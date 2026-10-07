#ifndef __PID_H__
#define __PID_H__
#include "stdint.h"

typedef struct
{    
	float Kp;                       // 比例系数
    float Ki;                       // 积分系数
    float Kd;                       // 微分系数
	float Kf;                       // 前馈系数

    float Target;                   // 目标值
    float Actual;                   // 实际值
    float Error;                    // 当前误差
    volatile float Integral;        // 积分值
    float IntegralLimit;            // 积分限幅
    float Output;                   // 输出值
	float Deadband;                 // 停止死区
	
    float LastTarget;               // 目标值
    float LastActual;               // 实际值
	float LastError;                // 上次误差
	
	float OutputLimit;              //输出限幅

    float LastDerivative;           //上一次微分输入
    float DerivativeLpf;            //滤波后的微分值
}PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float kf, float integral_limit,float output_limit,float deadband);
void PID_Update(PID_t *pid,float target,float measure);
float PID_POS_Update(PID_t *pid, float target_pos, float measure_pos);


#endif


