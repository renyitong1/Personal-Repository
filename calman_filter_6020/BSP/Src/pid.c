#include "pid.h"
#include "math.h"
#include <stdio.h>
#include "bsp_can.h"

float Difout;
float a = 0.8;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float kf, float integral_limit,float output_limit,float deadband)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
	pid->Kf = kf;
	
	pid->Target = 0;
    pid->Actual = 0;
    pid->Error = 0;
    pid->Integral = 0;
    pid->Output = 0;
	pid->Deadband = deadband;

    pid->IntegralLimit = integral_limit;
    pid->OutputLimit = output_limit;
    pid->LastTarget = 0;
    pid->LastActual = 0;
	pid->LastError = 0;

    pid->DerivativeLpf = 0.0f;
}    

//速度环Pid
void PID_Update(PID_t *pid,float target,float measure)
{
	pid->Target = target;
	pid->Actual = measure;
    pid->Error = pid->Target - pid->Actual;

    //误差死区
    if(fabs(pid->Error) <= pid->Deadband || pid->Target == 0.0f)
    {
        pid->Error = 0.0f;
		pid->Integral = 0.0f;
    }

    //积分分离
    if(fabs(pid->Error) < 500)
    {
        pid->Integral += pid->Error;
    }
    else
    {
        pid->Integral *= 0.98f;
    }
    
    // 积分限幅
    if (pid->Integral > pid->IntegralLimit) pid->Integral = pid->IntegralLimit;
    if (pid->Integral < -pid->IntegralLimit) pid->Integral = -pid->IntegralLimit;
    // PID计算
    float p_out = pid->Kp * pid->Error;
    float i_out = pid->Ki * pid->Integral;
    
    //更新：微分先行&低通滤波
    float alpha = 0.8f;
    //float gradient = ((pid->Target - pid->LastTarget) - (pid->Actual - pid->LastActual));
    float gradient = (pid->Actual - pid->LastActual);
	float d_raw = pid->Kd * gradient;
    pid->DerivativeLpf = alpha * pid->DerivativeLpf + (1.0f - alpha) * d_raw;
	float d_out = pid->DerivativeLpf;

//	Difout = (1 - a) * d_raw + a * Difout;

	float f_out = pid->Kf * pid->Target;
	
    pid->Output = f_out + p_out + i_out + d_out;
	
	 // 输出限幅
    if (pid->Output > pid->OutputLimit)  pid->Output = pid->OutputLimit;
    if (pid->Output < -pid->OutputLimit)  pid->Output = -pid->OutputLimit;
	
    pid->LastError = pid->Error;
	pid->LastTarget = pid->Target;
    pid->LastActual = pid->Actual;
}

//位置环pid
float PID_POS_Update(PID_t *pid, float target_pos, float measure_pos)
{
    pid->Target = target_pos;
	pid->Actual = measure_pos;
	
	pid->Error = pid->Target - pid->Actual;
	
	pid->Integral += pid->Error;
	
	if(pid->Integral > pid->IntegralLimit) pid->Integral = pid->IntegralLimit; 
	if(pid->Integral < -pid->IntegralLimit) pid->Integral = -pid->IntegralLimit; 
	
	float p_out = pid->Kp * pid->Error;
	float i_out = pid->Ki * pid->Integral;
	float d_out = pid->Kd * (pid->Error - pid->LastError);
	
	float output_pos = p_out + i_out + d_out;
	
	if(output_pos > pid->OutputLimit)  output_pos = pid->OutputLimit;
	if(output_pos < -pid->OutputLimit) output_pos = -pid->OutputLimit;

	pid->LastError = pid->Error;
	
	return output_pos;
}
