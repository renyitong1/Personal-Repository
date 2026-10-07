#include "usart.h"
#include "vofa_task.h"
#include "stdio.h"
#include "stdint.h"
#include "bsp_can.h"
#include "FreeRTOS.h"
#include "task.h" 

int fputc(int ch,FILE *f)
{
HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}

void send_vofa(void)
{                                                     
//    printf("%f,%f\r\n",
////       Motor_Spe[2].Target,
////       Motor_Spe[2].Actual,
////	   Motor_Spe[2].Kp,
////		w_input_filtered,
////		yaw.yaw_angle,
////		yaw.yaw_target
//	);
}

void vofa_task(void *argument)
{
 
	while(1)
	{
	    send_vofa();   
        vTaskDelay(10);

	}
}
