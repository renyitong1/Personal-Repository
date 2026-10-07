#ifndef __REMOTE_TASK_H
#define __REMOTE_TASK_H

#include "stdint.h"

void Parse_Data(uint8_t *buf);
void Uart_RX_Task(void *argument);
void Find_Frame(uint8_t *buf, uint16_t buf_len);
void remote_Task(void *argument);


#endif
