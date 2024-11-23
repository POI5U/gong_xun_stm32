#ifndef __RASPI_H__
#define __RASPI_H__

#include "usbd_cdc_if.h"
#include "system.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern uint8_t QR_code_array[20];
extern uint16_t Scan_cross_array[10];
extern uint16_t Scan_weight_array[10];
extern uint8_t mode_state;
extern uint8_t rec_sta;



#define RASPI_BUFFER_MAXSIZE 0xFF	

#define RASPI_TX_BUFFER_SIZE 0xFF
#define RASPI_RX_BUFFER_SIZE 512


#define RASPI_TASK_USE_STACK 512


typedef enum raspi_type{
	QR_start		= 1,
	QR_back			= 2,
	Ring_start		= 3,
	Ring_back		= 4,
	Weight_start	= 5,
	Weight_back		= 6,
	Continue_start	= 7,
	Continue_back	= 8,
	Yaw_Adjust_start= 9,
	Yaw_Adjust_back	=10
	
}raspi_type;


typedef struct raspi_tx_frame{	
	uint8_t send_buffer[RASPI_BUFFER_MAXSIZE];	// 发送数组
	uint32_t buffer_size;							// 发送数据长度
	uint32_t timeout;								// 等待计时超时上限 ( ms )
	
}raspi_tx_frame;


typedef struct raspi_data{
	uint8_t data_size;
	raspi_type type;
	uint8_t* data_buf;
	
} raspi_data;

void Send_Cmd(uint8_t* data, uint8_t size);
void USB_RxData(uint8_t* Buffer, uint32_t Recv_Num);
void Write_Data(raspi_type type, uint8_t* data_buf, uint8_t size);

void RASPI_Task(void* param);

#endif



