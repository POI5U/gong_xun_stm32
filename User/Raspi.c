#include "Raspi.h"
uint8_t QR_code_array[20] = {0};
uint16_t Scan_cross_array[10] = {0};
uint16_t Scan_weight_array[10] = {0};
uint8_t rec_sta;
uint8_t mode_state;

uint8_t RASPI_RXBUFFER[RASPI_RX_BUFFER_SIZE] = { 0 };
uint32_t raspi_rx_num = 0;




/**
 * @brief 用以作为中间层发送指令, 这里使用USBCDC, 一种可靠的协议, 
 * 不用考虑传输失败的问题
 */

// 私有
typedef enum RASPI_state{
	RASPI_STATE_READY = 0,
	RASPI_STATE_OFF = 1,
	RASPI_STATE_BUSY = 2,
	RASPI_STATE_WAITING = 3,
	
}RASPI_state;

static RASPI_state raspi_usb_state = RASPI_STATE_OFF;
static raspi_tx_frame usb_frame;
	

void Send_Cmd(uint8_t* data, uint8_t size)//static 
{
	// 等待usb准备好
	while( raspi_usb_state != RASPI_STATE_READY) {
		vTaskDelay(1);
	}
	
	memcpy(usb_frame.send_buffer, data, size);
	usb_frame.buffer_size = size;
	usb_frame.timeout = 500;
	
	raspi_usb_state = RASPI_STATE_BUSY;
	
}


/**
 * @brief 发送指令到树莓派
 */
void Write_Data(raspi_type type, uint8_t* data_buf, uint8_t buf_size)
{

	uint8_t send_data[RASPI_BUFFER_MAXSIZE] = { 0 };
	uint8_t cnt = 0;
	send_data[cnt++] = buf_size + 2;
	send_data[cnt++] = (uint8_t)type;
	
	for(uint8_t i = 0; i < buf_size; i++){
		send_data[cnt++] = data_buf[i];
	}
	Send_Cmd(send_data, cnt);
}


/**
 * @brief 在这个回调函数内处理各个信号的接收
 */
// 朱豪写的
int ascll(uint8_t num){
	return num - '0';
}

// 也是朱豪写的
void Raspi_Callback(uint8_t* now_buffer)
{
	int32_t now_size = now_buffer[0];
	print("-type: %d, len: %d, data:", now_buffer[1], now_buffer[0]);
	if(now_buffer[1] == QR_back){		
		for(uint8_t xset = 0; xset < 3; xset++){
			QR_code_array[xset] = ascll(now_buffer[xset+2]);
		}
		for(uint8_t yset = 3; yset < 6; yset++){
			QR_code_array[yset] = ascll(now_buffer[yset+3]);
		}
		print("%d%d%d+%d%d%d",QR_code_array[0], QR_code_array[1], QR_code_array[2],
								QR_code_array[3], QR_code_array[4], QR_code_array[5]);
		
	}else if(now_buffer[1] == Ring_back && now_buffer[0] == 13){
		Scan_cross_array[0] = ascll(now_buffer[4])*100 + ascll(now_buffer[5])*10 + ascll(now_buffer[6]);//x
		Scan_cross_array[1] = ascll(now_buffer[10])*100 + ascll(now_buffer[11])*10 + ascll(now_buffer[12]);//y
		print("%d %d\r\n", Scan_cross_array[0], Scan_cross_array[1]);
	}else if(now_buffer[1] == Weight_back){
		if(now_buffer[0] == 14){
		Scan_weight_array[0] = ascll(now_buffer[4])*100 + ascll(now_buffer[5])*10 + ascll(now_buffer[6]);//x
		Scan_weight_array[1] = ascll(now_buffer[10])*100 + ascll(now_buffer[11])*10 + ascll(now_buffer[12]);//y
		Scan_weight_array[2] = now_buffer[13] == 'r' ? 1:(now_buffer[13] == 'g' ? 2:3);
		print("%d %d color:%d\r\n", Scan_weight_array[0], Scan_weight_array[1], Scan_weight_array[2]);
		mode_state = 1;
		}else{
			Scan_weight_array[2] = 0;
			mode_state = 1;
			print("have no mode");
		}
	}else if(now_buffer[1] == Continue_back){
		print("02 08\r\n");
	}else if(now_buffer[1] == Yaw_Adjust_back){
		int yaw_data = 0;
		yaw_data = (now_buffer[2] << 0) | (now_buffer[3] << 8) | (now_buffer[4] << 16) | (now_buffer[5] << 24);
		float *p_yaw = (float*)(&yaw_data);
		yaw += *p_yaw;
	}
	rec_sta = 1;
	print("\r\n");
}


// 加入缓冲区
void USB_RxData(uint8_t* Buffer, uint32_t Recv_Num)
{
	memcpy(&RASPI_RXBUFFER[raspi_rx_num], Buffer, Recv_Num);
	// 更新缓冲区中数据的总长度
	raspi_rx_num += Recv_Num;
	
}


void RASPI_Task(void* param)
{
	const int32_t tick_ms = 1; // 1ms执行一次
	const TickType_t xFrequency = pdMS_TO_TICKS(tick_ms);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	uint32_t time_add = 0;		// 超时累计积分
	raspi_usb_state = RASPI_STATE_READY;
	uint32_t data_len = 0;
	
	while(1) {
		switch( raspi_usb_state) {
			case RASPI_STATE_READY:
			case RASPI_STATE_OFF:
				break;
			
			case RASPI_STATE_BUSY: {
				while( CDC_Transmit_HS(usb_frame.send_buffer, usb_frame.buffer_size)!= USBD_OK) {
					vTaskDelay(1);
				}
				vTaskDelay(2);	// 等待发送完全
				time_add = 0;
				data_len = 0;
				raspi_usb_state = RASPI_STATE_WAITING;
				break;
			}
			case RASPI_STATE_WAITING: {
				time_add += tick_ms;
				// 超时判断 重新发送一次
				if( time_add >= usb_frame.timeout) {
					raspi_usb_state = RASPI_STATE_BUSY;
				}
				// 接收到指定数量
				if( raspi_rx_num != 0 && raspi_rx_num >= RASPI_RXBUFFER[0] ) {
					Raspi_Callback(RASPI_RXBUFFER);
					vTaskDelay(10);	// 等待总线冷却
					
					raspi_rx_num = 0;
					raspi_usb_state = RASPI_STATE_READY;
				}
				break;
			}
		}
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	
}


