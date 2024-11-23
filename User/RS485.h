#ifndef __RS485_H__
#define __RS485_H__

#include "system.h"

/*
目标接收字节数, 目标超时计数

*/

#define RS485_SEND_BUFFER_SIZE 256

#define RS485_TASK_USE_STACK 1024



typedef struct rs485_tx_frame{
	
	uint8_t send_buffer[RS485_SEND_BUFFER_SIZE];	// 发送数组
	uint32_t buffer_size;							// 发送数据长度
	uint32_t recv_size;								// 预计接收数量
	uint32_t timeout;								// 等待计时超时上限 ( ms )
	void (*rx_callback_func)(uint8_t* recv_data, uint32_t recv_size);	// 接收数据处理函数
	uint8_t (*rx_check_func)(uint8_t* recv_data, uint32_t recv_size);	// 检查数据时候合法函数
	
}rs485_tx_frame;


HAL_StatusTypeDef RS_485_send(rs485_tx_frame* frame);
void RS485_Task(void* param);
HAL_StatusTypeDef RS485_get_state(void);


#endif





