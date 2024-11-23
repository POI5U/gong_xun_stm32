#include "LK_motor.h"
#include "system.h"
#include "arm.h"
#include "RS485.h"

extern Arm RoboArm;

static void LK_do_nothing(uint8_t* rdata, uint32_t size)
{
	return ;
}

static uint8_t Calc_add(uint8_t* data, uint8_t num)
{
	uint8_t add = 0;
	for(int i = 0; i < num; i++){
		add += data[i];
	}
	return add;
}


static uint8_t LK_check_data(uint8_t* rdata, uint32_t size)
{
	// 数据域长度
	uint8_t len_data_zone = rdata[3];
	// 帧头校验
	if( Calc_add(rdata, 4) != rdata[4]) {
		return 0;
	}
	// 无数据域则帧头校验成功就成功
	if( len_data_zone == 0) {
		return 1;
	}
	// 校验数据域
	if( Calc_add(rdata + 5, len_data_zone) != rdata[size - 1]) {
		return 0;
	}
	// 所有都校验成功
	return 1;
}



HAL_StatusTypeDef LK_motor_disable(LK_motor* motor)
{
	rs485_tx_frame Cmd = {0};
	Cmd.send_buffer[0] = 0x3E;
	Cmd.send_buffer[1] = 0x81;
	Cmd.send_buffer[2] = motor->id;
	Cmd.send_buffer[3] = 0x00;
	Cmd.send_buffer[4] = Calc_add(Cmd.send_buffer, 4);
	
	Cmd.buffer_size = 5;
	Cmd.recv_size = 5;
	Cmd.rx_callback_func = LK_do_nothing;
	Cmd.rx_check_func = LK_check_data;
	Cmd.timeout = 50;
	
	return RS_485_send(&Cmd);
}

static void LK_motor_read_angle_callback(uint8_t* recv_data, uint32_t recv_size)
{
	uint32_t motorangleL = (recv_data[5]<<0)| (recv_data[6]<<8)|
			(recv_data[7]<<16)|(recv_data[8]<<24);
	uint32_t motorangleH = (recv_data[9]<<0)|(recv_data[10]<<8)|
			(recv_data[11]<<16)|(recv_data[12]<<24);
	
	int64_t nowPU = (((int64_t)motorangleH << 32)|motorangleL);
	
	if(recv_data[2] == RoboArm.L_motor.id){
		RoboArm.L_motor.nowPU = nowPU;
	}else if(recv_data[2] == RoboArm.R_motor.id){
		RoboArm.R_motor.nowPU = nowPU;
	}else if(recv_data[2] == RoboArm.U_motor.id){
		RoboArm.U_motor.nowPU = nowPU;
	}
}

HAL_StatusTypeDef LK_motor_read_angle(LK_motor* motor)
{
	rs485_tx_frame Cmd = {0};
	Cmd.send_buffer[0] = 0x3E;
	Cmd.send_buffer[1] = 0x92;
	Cmd.send_buffer[2] = motor->id;
	Cmd.send_buffer[3] = 0x00;
	Cmd.send_buffer[4] = Calc_add(Cmd.send_buffer, 4);
	
	Cmd.buffer_size = 5;
	Cmd.recv_size = 14;
	Cmd.rx_callback_func = LK_motor_read_angle_callback;
	Cmd.rx_check_func = LK_check_data;
	Cmd.timeout = 50;
	
	return RS_485_send(&Cmd);
}


HAL_StatusTypeDef LK_Position_Control(LK_motor* motor)
{
	rs485_tx_frame Cmd = {0};
	Cmd.send_buffer[0] = 0x3E;
	Cmd.send_buffer[1] = 0xA4;
	Cmd.send_buffer[2] = motor->id;
	Cmd.send_buffer[3] = 0x0C;
	Cmd.send_buffer[4] = Calc_add(Cmd.send_buffer, 4);
	
	for(int i=0;i<8;i++){
		Cmd.send_buffer[5 + i] = ((int64_t)motor->goalPU >> (i * 8)) & 0xFF;
		Cmd.send_buffer[17] += Cmd.send_buffer[5+i];
	}
	
	for(int i=0;i<4;i++){
		Cmd.send_buffer[13 + i] = ((int32_t)motor->maxspeed >> (i * 8)) & 0xFF;
		Cmd.send_buffer[17] += Cmd.send_buffer[13 + i];
	}
	
	Cmd.buffer_size = 18;
	Cmd.recv_size = 13;
	Cmd.rx_callback_func = LK_do_nothing;
	Cmd.rx_check_func = LK_check_data;
	Cmd.timeout = 50;
	
	return RS_485_send(&Cmd);
}



