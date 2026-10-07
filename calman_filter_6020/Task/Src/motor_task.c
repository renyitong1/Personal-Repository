#include "can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h" 
#include <stdio.h>
#include "math.h"
#include "motor_task.h"
#include "pid.h"
#include "bsp_can.h"
#include "remote_task.h"

#define RC_t  0.04f   // 时间常数（反应速度）
#define DT    0.01f   // 任务周期（100Hz）

PID_t Motor_Spe[4] = {0};
int16_t cur[4] = {0};
float V[4] = {0,0,0,0};

extern motor_measure_t moto_chassis[8];
extern int16_t Channel[5];

float deadband(float x, float db)
{
    if (x > db)      return x - db;
    else if (x < -db) return x + db;
    else return 0;
}

void Speed_Calc(void)
{
	float raw_Vx = deadband(Channel[2],60);
	float Vx_f;
	Vx_f += (raw_Vx - Vx_f) * (DT / (RC_t + DT));
	V[0] = Vx_f;
	
	PID_Update(&Motor_Spe[0], V[0], moto_chassis[0].speed_rpm * 25000 / 320);
	cur[0] = (int16_t)Motor_Spe[0].Output;
//	if(Channel[2] == 0)
//	{
//		cur[0] = 0;
//	}
	send_chassis_cur1_4(cur[0], 0, 0, 0);
}

void motor_task(void *argument)
{
	CAN_Filter_Init();

	PID_Init(&Motor_Spe[0], 3.0f, 0.0f, 3.0f, 0.30f, 25000.0f, 25000.0f, 30);
	
	while(1)
	{
		Speed_Calc();
		vTaskDelay(10);
	}
	
}
	
