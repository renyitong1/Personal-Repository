#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "main.h"
#include "can.h"
#include "pid.h"

//3508电机
typedef struct{
	uint16_t angle;//电机转子机械角度
	uint16_t last_angle;//电机上一次转子机械角度
	int32_t total_angle;//电机总角度
	float last_current;//电机的上一次电流
	float real_current;//电流
	int16_t speed_rpm;//电机转子速度	
	int16_t speed_rpm_filtered;
	float total_angle_output;
}motor_measure_t;

//ID
typedef enum{
	
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_2006_M1_ID = 0x201,
	CAN_2006_M2_ID = 0x202,
	CAN_2006_M3_ID = 0x203,
	CAN_2006_M4_ID = 0x204,
	
	CAN_6020_ALL_ID = 0x1FF,
    CAN_2006_M5_ID = 0x205,
	CAN_2006_M6_ID = 0x206,
	CAN_2006_M7_ID = 0x207,
	CAN_2006_M8_ID = 0x208,    // 6020电机4反馈ID
    CAN_6020_ANGLE_MAX = 8192, 
	CAN_6020_MAX_CURRENT =3000,
}ID;

typedef struct
{
    uint16_t can_id;//电机ID
    int16_t  set_voltage;//设定的电压值
    uint16_t rotor_angle;//机械角度
    int16_t  rotor_speed;//转速
    int16_t  torque_current;//扭矩电流
    uint8_t  temp;//温度
}moto_info_t;

void CAN_Filter_Init(void);

void send_chassis_cur1_4(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


#endif

