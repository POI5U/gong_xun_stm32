#include "motor_midware.h"


uint8_t* Arm_Recv_Buffer = Uart2_RecvBuffer;
uint8_t *Arm_Recv_num = (uint8_t*)&Uart2_RecvNum;


void motor_delayms(uint32_t ms)
{
	HAL_Delay(ms);
}


HAL_StatusTypeDef arm_sendcmd(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	HAL_StatusTypeDef state = HAL_UART_Transmit(&huart2, pData, Size, Timeout);
	HAL_Delay(10);
	return state;
}


