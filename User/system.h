#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "stdlib.h"

#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

#include "ws2812.h"
#include "BMI088driver.h"
#include "IMU.h"
#include "Raspi.h"
#include "shell.h"
#include "EMM_motor.h"
#include "LK_motor.h"
#include "tim.h"
#include "RS485.h"




#define SYSTEM_TASK_USE_STACK 128



#define UART1_SEND_BUFFERSIZE 256

#define UART2_RECV_BUFFERSIZE 256
#define RS485_RECV_BUFFERSIZE 256

enum LED_STATE{
	//		0xABCDEF : AB: 红色, CD: 绿色, EF: 蓝色
	LED_OFF = 0x000000,
	
	LED_GREEN_WAKE = 0x000F00,
	LED_GREEN = 0x00FF00,
	
	LED_RED_WAKE = 0x0F0000,
	LED_RED = 0xFF0000,
	
	LED_BLUE_WAKE = 0x00000F,
	LED_BLUE = 0x0000FF,
	
	LED_WHITE_WAKE = 0x0F0F0F,
	LED_WHITE = 0xFFFFFF,
	
};



extern uint8_t Uart2_RecvBuffer[UART2_RECV_BUFFERSIZE];
extern uint32_t Uart2_RecvNum;


extern uint8_t RS485_RecvBuffer[RS485_RECV_BUFFERSIZE];
extern uint32_t RS485_RecvNum;




void usDelay(uint32_t us);
void creat_task(void);
void User_Hardware_Init(void);
void print(const char* format, ...);
void set_run_state(enum LED_STATE state);



#endif







