#include "RS485.h"

// 私有
typedef enum RS_485_state{
	RS485_STATE_READY = 0,
	RS485_STATE_OFF = 1,
	RS485_STATE_BUSY = 2,
	RS485_STATE_WAITING = 3,
	
}RS_485_state;


static RS_485_state rs485_2 = RS485_STATE_WAITING;
static rs485_tx_frame using_frame;

extern uint8_t RS485_RecvBuffer[RS485_RECV_BUFFERSIZE];
extern uint32_t RS485_RecvNum;

HAL_StatusTypeDef RS_485_send(rs485_tx_frame* frame)
{
	if( rs485_2 != RS485_STATE_READY) {
		return HAL_ERROR;
	}
	
	rs485_2 = RS485_STATE_BUSY;
	memcpy(&using_frame, frame, sizeof(rs485_tx_frame));
	return HAL_OK;
}


HAL_StatusTypeDef RS485_get_state(void)
{
	if( rs485_2 == RS485_STATE_READY) {
		return HAL_OK;
	}else {
		return HAL_ERROR;
	}
}

void RS485_Task(void* param)
{
	const int32_t tick_ms = 1; // 1ms执行一次
	const TickType_t xFrequency = pdMS_TO_TICKS(tick_ms);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	uint32_t time_add = 0;		// 超时累计积分
	rs485_2 = RS485_STATE_READY;
	
	while(1) {
		switch( rs485_2) {
			case RS485_STATE_READY:
			case RS485_STATE_OFF:
				break;
			
			case RS485_STATE_BUSY: {
				HAL_UART_Transmit_IT(&huart2, using_frame.send_buffer, using_frame.buffer_size);
				vTaskDelay(2);	// 等待发送完全
				time_add = 0;
				rs485_2 = RS485_STATE_WAITING;
				break;
			}
			case RS485_STATE_WAITING: {
				time_add += tick_ms;
				// 超时判断 重新发送一次
				if( time_add >= using_frame.timeout) {
					rs485_2 = RS485_STATE_BUSY;
				}
				// 接收到指定字节
				if( RS485_RecvNum >= using_frame.recv_size) {
					// 检查未通过
					if( using_frame.rx_check_func(RS485_RecvBuffer, using_frame.recv_size) != 1) {
						vTaskDelay(10);	// 等待总线冷却
						RS485_RecvNum = 0;
						rs485_2 = RS485_STATE_BUSY;
						break;
					}
					// 检查通过
					using_frame.rx_callback_func(RS485_RecvBuffer, using_frame.recv_size);
					vTaskDelay(10);	// 等待总线冷却
					
					RS485_RecvNum = 0;
					rs485_2 = RS485_STATE_READY;
				}
				break;
			}
		}
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	
	
}



