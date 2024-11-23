#ifndef __MOTOR_MIDWARE__
#define __MOTOR_MIDWARE__


#include "system.h"


extern uint8_t* Arm_Recv_Buffer;
extern uint8_t *Arm_Recv_num;


void motor_delayms(uint32_t ms);



HAL_StatusTypeDef arm_sendcmd(uint8_t *pData, uint16_t Size, uint32_t Timeout);



#endif
