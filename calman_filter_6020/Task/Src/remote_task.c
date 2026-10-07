#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "usart.h"
#include "semphr.h"
#include "remote_task.h"

extern osSemaphoreId_t RX_SemaphoreHandle;

//帧头:所有数据通用
#define FRAME_HEAD1 0xFF
#define FRAME_HEAD2 0x55

#define CMDID_HIGH  0x01
#define CMDID_LOW   0x01

#define DATA_LEN    0x17

//接收到的数据字节数
static uint16_t dSize;

uint8_t RX_Buf[64];

int16_t Channel[5];
int16_t Key_State[13];

void Find_Frame(uint8_t *buf, uint16_t buf_len)
{
  uint16_t i = 0;
  //buf_len至少要大于等于帧头帧尾加校验位加数据位长度
  while(i + 4 <= buf_len)
  {
      if(buf[i] == FRAME_HEAD1 && buf[i + 1] == FRAME_HEAD2)
      {
        uint8_t len = buf[4 + i];
	      if(i + 4 + len > buf_len)
	      {
	        break;
	      }
	      Parse_Data(&buf[i]);
	      i += (4 + len);
      }
      else
      {
        i ++;
      }
 }
}

//解析数据，提取摇杆数据和按键状态
void Parse_Data(uint8_t *buf)
{
  static int16_t Key_Data[13];
  if(buf[2] == CMDID_HIGH && buf[3] == CMDID_LOW 
	 && buf[4] == DATA_LEN)
  {  
      int16_t Rocker_Data[5] = {
      buf[6] << 8 | buf[5],
      buf[8] << 8 | buf[7],
      buf[10] << 8 | buf[9],
      buf[12] << 8 | buf[11],
      buf[14] << 8 | buf[13],
  };
  memcpy(Channel, Rocker_Data, sizeof(Rocker_Data));
  for(int i = 0; i < 13; i ++)
  {
    Key_Data[i] = buf[15 + i];
  }
   memcpy(Key_State, Key_Data, sizeof(Key_Data));
 }
}

HAL_StatusTypeDef State;

void Uart_RX_Task(void *argument)
{
    State = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX_Buf, sizeof(RX_Buf));
	
    //xSemaphoreGive(RX_SemaphoreHandle);
    while(1)
    {
      if(xSemaphoreTake(RX_SemaphoreHandle, portMAX_DELAY) == pdTRUE)
      {
          Find_Frame(RX_Buf, dSize);
      }
	    vTaskDelay(10);
    }
}

void remote_Task(void *argument)
{
    State = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX_Buf, sizeof(RX_Buf));
	
    //xSemaphoreGive(RX_SemaphoreHandle);
    while(1)
    {
      if(xSemaphoreTake(RX_SemaphoreHandle, portMAX_DELAY) == pdTRUE)
      {
          Find_Frame(RX_Buf, dSize);
      }
	    vTaskDelay(10);
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART2)
    {
        //上下文切换标志位
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        //释放信号量
        xSemaphoreGiveFromISR(RX_SemaphoreHandle, &xHigherPriorityTaskWoken);
        //本次接收到的实际有效数据字节数
        dSize = Size;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RX_Buf, sizeof(RX_Buf));
        //上下文切换，任务调度
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

