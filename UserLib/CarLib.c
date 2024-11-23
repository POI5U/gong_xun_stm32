#include "CarLib.h"


static const float Pi = 3.14159f;			// pi的值
static const int16_t angle_max = 0x2000;	// 单圈角度最大值(无法达到的最大值)
static const float Rround = 76.0f;			// 轮子直径(mm)
static const int16_t _retio = 36;			// 减速比
static const double Rangle = 0.01745329252;	// 幅度转角度



MiniCar Dcar = { 0 };



void Carlib_Init(float thread_mm)
{
	memset(&Dcar, 0, sizeof(MiniCar));
	
	//PID_Init(&Dcar.pos_pid, POS_PID, 0.007, 0, 0.04, 1000, 0, 50);
	Pos_speed_Init(&Dcar.pos_close, 1200, 50, 1200, 2100, 0.4, 0.78, 0.99, 0.008);
	
	PID_Init(&Dcar.angle_pid, POS_PID, 22, 0.001, 0, 500, 0, 50);
	Dcar.Car_State = ANGLE_ONLY;
	Dcar.get_thread_PU = thread_mm * _retio * angle_max / Rround / Pi;
	
	
	Dcar.reatch_thread = 0;
	
	Dcar.reatch_thread_max = 3;
}



void Car_turn_set(float goal_angle)
{
	Dcar.Car_State = ANGLE_ONLY;
	Dcar.goal_angle = goal_angle;
}



void Car_move_set(float x_pos_mm, float y_pos_mm)
{
	Dcar.reatch_thread = 0;
	Dcar.Car_State = ANGLE_AND_POS;
	float delta_x = x_pos_mm - Dcar.goal_x_mm;
	float delta_y = y_pos_mm - Dcar.goal_y_mm;
	
	Dcar.goal_y_mm = y_pos_mm;
	Dcar.goal_x_mm = x_pos_mm;
	
	double car_x = delta_x * cos(Dcar.goal_angle * Rangle)
		+ delta_y * sin(Dcar.goal_angle * Rangle);
	double car_y = delta_y * cos(Dcar.goal_angle * Rangle)
		- delta_x * sin(Dcar.goal_angle * Rangle);
	
	Dcar._PU_add = sqrt( pow(car_x, 2) + pow(car_y, 2) )
	* _retio * angle_max / Rround / Pi;
	
	Dcar._PU_add_angle = atan2(car_y, car_x);
	
	Dji_reset_pos();
	Dcar.pos_close.last_speed = 0;
}




void Car_calc(float now_yaw, float dt_ms)
{
	int32_t x_have_go = (LU_Motor._all_angle - LD_Motor._all_angle +
		RU_Motor._all_angle - RD_Motor._all_angle) / 4;

	int32_t y_have_go = (LU_Motor._all_angle + LD_Motor._all_angle -
		RU_Motor._all_angle - RD_Motor._all_angle) / 4;

	int32_t have_going = sqrtf( powf(x_have_go, 2) + powf(y_have_go, 2) );

	//PID_Calc(&Dcar.pos_pid, Dcar._PU_add - have_going, dt_ms);
	int32_t pos_speed = Pos_speed_Calc(&Dcar.pos_close, Dcar._PU_add, have_going, dt_ms);
	int32_t turn_speed = PID_Calc(&Dcar.angle_pid, Dcar.goal_angle - now_yaw, dt_ms);

	int32_t pos_x_speed = cos(Dcar._PU_add_angle) * pos_speed;
	int32_t pos_y_speed = sin(Dcar._PU_add_angle) * pos_speed;

	if( pos_speed < 3 && abs(Dcar._PU_add - have_going) < Dcar.get_thread_PU){
		Dcar.reatch_thread++;
	} else {
		Dcar.reatch_thread = 0;
	}
	
	if( Dcar.reatch_thread >= Dcar.reatch_thread_max) {
		Dcar.reatch_thread = 0;
		Dcar.Car_State = ANGLE_ONLY;
	}

	LU_Motor.goal_speed = - turn_speed;
	LD_Motor.goal_speed = - turn_speed;
	RU_Motor.goal_speed = - turn_speed;
	RD_Motor.goal_speed = - turn_speed;

	if( Dcar.Car_State != ANGLE_ONLY) {
		LU_Motor.goal_speed += + pos_x_speed + pos_y_speed;
		LD_Motor.goal_speed += - pos_x_speed + pos_y_speed;
		RU_Motor.goal_speed += + pos_x_speed - pos_y_speed;
		RD_Motor.goal_speed += - pos_x_speed - pos_y_speed;
	}

}


void Car_set_xy(float new_x, float new_y)
{
	Dcar.goal_x_mm = new_x;
	Dcar.goal_y_mm = new_y;
}


Car_PID_State Car_check(void)
{
	return Dcar.Car_State;
}

