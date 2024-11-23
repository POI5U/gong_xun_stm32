#include "EMM_motor.h"
#include "RS485.h"
#include "system.h"
#include "arm.h"
#include "CRC.h"

extern Arm RoboArm;

static void EMM_do_nothing(uint8_t* rdata, uint32_t size)
{
	return ;
}

static uint8_t Emm_check_data(uint8_t* rdata, uint32_t size)
{
	if( CRC8(rdata, size - 1) == rdata[size - 1]) {
		return 1;
	}else {
		return 0;
	}
}


HAL_StatusTypeDef EMM_set_position(EMM_motor* motor)		
{
	rs485_tx_frame Cmd = {0};
	int32_t PU;
	Cmd.send_buffer[2] = 0x00;	//cw 设置为1即为反向

	if(motor->state == 0){
		PU = EMM_UNLOCK_POS;
	}
	else if(motor->state == 1){
		PU = EMM_LOCK_POS;
	}
	Cmd.send_buffer[0] = motor->id;      
	Cmd.send_buffer[1] = 0xFD;
	Cmd.send_buffer[3] = (motor->maxspeed >> 8 ) & 0xFF;
	Cmd.send_buffer[4] = (motor->maxspeed >> 0 ) & 0xFF;

	Cmd.send_buffer[5] = 0x00;
	
	Cmd.send_buffer[6] = (PU >> 24) & 0xFF;
	Cmd.send_buffer[7] = (PU >> 16) & 0xFF;
	Cmd.send_buffer[8] = (PU >>  8) & 0xFF;
	Cmd.send_buffer[9] = (PU >>  0) & 0xFF;

	Cmd.send_buffer[10] = 0x01;		
	Cmd.send_buffer[11] = 0x00;
	Cmd.send_buffer[12] = CRC8(Cmd.send_buffer, 12);
	
	Cmd.buffer_size = 13;
	Cmd.recv_size = 4;
	Cmd.rx_callback_func = EMM_do_nothing;
	Cmd.rx_check_func = Emm_check_data;
	Cmd.timeout = 50;
	
	return RS_485_send(&Cmd);
}


void EMM_read_position_callback(uint8_t* recv_data, uint32_t recv_size)
{
	if( recv_data[2] == 0x00) {
		RoboArm.Paw.position = (int32_t)((recv_data[3]<<24) | (recv_data[4]<<16)|
			(recv_data[5]<<8) | (recv_data[6]<<0));
	}else if( recv_data[2] == 0x01) {
		RoboArm.Paw.position = -(int32_t)((recv_data[3]<<24) | (recv_data[4]<<16)|
			(recv_data[5]<<8) | (recv_data[6]<<0));
	}
}

HAL_StatusTypeDef EMM_read_position(EMM_motor* motor)
{
	rs485_tx_frame Cmd = {0};

	Cmd.send_buffer[0] = motor->id;      
	Cmd.send_buffer[1] = 0x36;
	Cmd.send_buffer[2] = CRC8(Cmd.send_buffer, 2); 
	
	Cmd.buffer_size = 3;
	Cmd.recv_size = 8;
	Cmd.rx_callback_func = EMM_read_position_callback;
	Cmd.rx_check_func = Emm_check_data;
	Cmd.timeout = 50;

	return RS_485_send(&Cmd);
}


HAL_StatusTypeDef EMM_motor_enable(EMM_motor* motor, Emm_lock State)
{
	rs485_tx_frame Cmd = {0};
	Cmd.send_buffer[0] = motor->id;      
	Cmd.send_buffer[1] = 0xF3;
	Cmd.send_buffer[2] = 0xAB;     
	Cmd.send_buffer[3] = State;
	Cmd.send_buffer[4] = 0x00;     
	Cmd.send_buffer[5] = CRC8(Cmd.send_buffer, 5);
	
	Cmd.buffer_size = 6;
	Cmd.recv_size = 4;
	Cmd.rx_callback_func = EMM_do_nothing;
	Cmd.rx_check_func = Emm_check_data;
	Cmd.timeout = 50;
	
	return RS_485_send(&Cmd);
}


Emm_lock EMM_judge_pos(EMM_motor* motor)
{
	EMM_read_position(motor);
	vTaskDelay(50);
	
	if( RoboArm.Paw.position >= ((EMM_UNLOCK_POS + EMM_LOCK_POS) / 2) ){
		return PAW_LOCK;
	}
	return PAW_UNLOCK;
}
