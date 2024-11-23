#include "DJI_motor.h"


static const float Pi = 3.14159f;			// pi的值
static const int16_t angle_max = 0x2000;	// 单圈角度最大值(无法达到的最大值)
static const float _speed_x = 0.6f;			// 速度滤波系数(越大越平滑, 也越滞后)
static const float Rround = 76.0f;			// 轮子直径(mm)


DJI_motor LU_Motor = { 0 };		// id = 1
DJI_motor LD_Motor = { 0 };		// id = 2		
DJI_motor RU_Motor = { 0 };		// id = 3
DJI_motor RD_Motor = { 0 };		// id = 4

static void singel_motor_init(DJI_motor* motor, int8_t id)
{
	memset(motor, 0, sizeof(DJI_motor));
	motor->id = id;
	motor->goal_speed = 0;
	motor->_retio = 36;
}


void Dji_Init(void)
{
	
	singel_motor_init(&LU_Motor, 1);
	singel_motor_init(&LD_Motor, 2);
	singel_motor_init(&RU_Motor, 3);
	singel_motor_init(&RD_Motor, 4);
	//130, 10, 0, 10000, 10, 60);
	PID_Init(&LU_Motor.pid_speed, POS_PID, 50, 11, 0.5, 10000, 10, 50);
	PID_Init(&LD_Motor.pid_speed, POS_PID, 50, 11, 0.5, 10000, 10, 50);
	PID_Init(&RU_Motor.pid_speed, POS_PID, 50, 11, 0.5, 10000, 10, 50);
	PID_Init(&RD_Motor.pid_speed, POS_PID, 50, 11, 0.5, 10000, 10, 50);

}


void Dji_recv(DJI_motor* motor, Can_Frame* frame)
{
	motor->__last_singel_angle = motor->__singel_angle;
	
	// 更新当前状态
	motor->__singel_angle =	(frame->frame_data[0] << 8) | frame->frame_data[1];
	motor->_now_speed 	  = (frame->frame_data[2] << 8) | frame->frame_data[3];
	motor->__real_force	  = (frame->frame_data[4] << 8) | frame->frame_data[5];
	
	// 以变化最小的为准
	int32_t _angle_delta = motor->__singel_angle - motor->__last_singel_angle;
	int32_t _other_delta = _angle_delta > 0 ? _angle_delta - angle_max : _angle_delta + angle_max;
	
	motor->_all_angle += (abs(_other_delta) < abs(_angle_delta)) ? _other_delta : _angle_delta;

	// 车速度 (mm / s)
	float out_speed = motor->_now_speed * Pi * Rround / motor->_retio / 60.0f;
	// 实际输出速度平滑滤波
	motor->real_speed = out_speed * (1 - _speed_x) + _speed_x * motor->real_speed;
}



void Dji_put_speed(void)
{
	Can_Frame send_frame = { 0 };
	send_frame.data_len = 8;
	send_frame.frame_id = 0x200;
	
	DJI_motor motors[4] = { LU_Motor, LD_Motor, RU_Motor, RD_Motor };
	for( int i = 0; i < 4; i++) {
		send_frame.frame_data[i * 2] = (motors[i].set_force >> 8) & 0xFF;
		send_frame.frame_data[i * 2 + 1] = motors[i].set_force & 0xFF;
	}

	fdcanx_send_frame(&hfdcan1, &send_frame);
}


void Dji_motor_calc(float dt)
{

	LU_Motor.set_force = PID_Calc(&LU_Motor.pid_speed, LU_Motor.goal_speed - LU_Motor.real_speed, dt);
	LD_Motor.set_force = PID_Calc(&LD_Motor.pid_speed, LD_Motor.goal_speed - LD_Motor.real_speed, dt);
	RU_Motor.set_force = PID_Calc(&RU_Motor.pid_speed, RU_Motor.goal_speed - RU_Motor.real_speed, dt);
	RD_Motor.set_force = PID_Calc(&RD_Motor.pid_speed, RD_Motor.goal_speed - RD_Motor.real_speed, dt);

}


void Dji_reset_pos(void)
{
	LU_Motor._all_angle = 0;
	LD_Motor._all_angle = 0;
	RU_Motor._all_angle = 0;
	RD_Motor._all_angle = 0;
}


