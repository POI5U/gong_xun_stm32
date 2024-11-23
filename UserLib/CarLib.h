#ifndef __CARLIB_H__
#define __CARLIB_H__


#include "stdint.h"
#include "string.h"
#include "math.h"

#include "pid.h"

#include "DJI_motor.h"

typedef enum Car_PID_State{
	ANGLE_ONLY,
	ANGLE_AND_POS,

}Car_PID_State;





typedef struct MiniCar{
	float goal_angle;

	float goal_x_mm;
	float goal_y_mm;
	
	int32_t _PU_add;		// 移动PU增加的总位置
	float _PU_add_angle;	// 移动PU的角度
	
	int32_t get_thread_PU;	// 检测达到的阈值PU
	int32_t reatch_thread;	// 到达计数
	int32_t reatch_thread_max;	// 到达计数最大值, 达到则达到位置
	
	
	pos_speed_control pos_close;	// 位置输出速度计算结构体
	PID_t angle_pid;
	
	Car_PID_State Car_State;
}MiniCar;



void Carlib_Init(float thread_mm);
void Car_turn_set(float goal_angle);
void Car_move_set(float x_pos_mm, float y_pos_mm);
void Car_calc(float now_yaw, float dt_ms);
Car_PID_State Car_check(void);
void Car_set_xy(float new_x, float new_y);

#endif



