#include "bsp_can.h"
#include "math.h"
#include <stdlib.h> 
#include "can.h"

#define ALPHA 0.3f
#define M3508_ENCODER_RESOLUTION 8192 //编码器端
#define M3508_REDUCTION_RATIO (3591.0f / 187.0f)  // 减速比 ≈ 19.19

extern CAN_HandleTypeDef hcan1;

CAN_TxHeaderTypeDef  chassis_tx_message;
uint8_t chassis_can_send_data[8] = {0};  //发送数据缓冲
uint8_t data_current[8] = {0};

motor_measure_t moto_chassis[8] = {0};
moto_info_t motor_yaw_info;

//CAN过滤器初始化
void CAN_Filter_Init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_st.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//3508电机控制指令发送函数
void send_chassis_cur1_4(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box; //发送邮箱
	
	chassis_tx_message.StdId = CAN_6020_ALL_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	
	data_current[0] = motor1 >> 8;
	data_current[1] = motor1;
	data_current[2] = motor2 >> 8;
	data_current[3] = motor2;
	data_current[4] = motor3 >> 8;
	data_current[5] = motor3;
	data_current[6] = motor4 >> 8;
	data_current[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, data_current, &send_mail_box);
}

//获取总角度
void get_total_angle(motor_measure_t *p)
{
  int res1,res2,delta;
	
  if(p->angle < p->last_angle)
 {
  res1 = p->angle - p->last_angle + M3508_ENCODER_RESOLUTION;  //正
  res2 = p->angle - p->last_angle;  //反
 }
  else
 {
  res1 = p->angle - p->last_angle - M3508_ENCODER_RESOLUTION;  //反
  res2 = p->angle - p->last_angle;  //正
 }
 if(abs(res1) < abs(res2))
 {delta = res1;}
 else
 {delta = res2;}
 
 p->total_angle += delta;
 p->last_angle = p->angle;
 
 p->total_angle_output = (float)p->total_angle / M3508_REDUCTION_RATIO;
}

// 解析3508电机反馈数据
void get_motor_measure(motor_measure_t *ptr, uint8_t *Data)
{
  ptr->last_angle = ptr->angle;
  ptr->angle = (uint16_t)(Data[0] << 8|Data[1]);
  
  int16_t raw_speed = (int16_t)(Data[2] << 8 | Data[3]);
  ptr->speed_rpm = raw_speed;

  int16_t current_raw = (int16_t)(Data[4] << 8 | Data[5]);
  ptr->real_current = current_raw * 5.0f / 16384.0f;
}

//回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8] = {0};
    uint16_t rec_id[4];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan->Instance == CAN1)
    {
		rec_id[0] = rx_header.StdId;
        // CAN1 -> 电机反馈
        switch(rec_id[0])
        {
            case CAN_2006_M5_ID:
            case CAN_2006_M6_ID:
            case CAN_2006_M7_ID:
            case CAN_2006_M8_ID:
                {
                    uint8_t i = rx_header.StdId - CAN_2006_M5_ID;
                    get_motor_measure(&moto_chassis[i], rx_data);
                    get_total_angle(&moto_chassis[i]);
                    break;
                }
				
            default:
                break;
        }
    }
}
