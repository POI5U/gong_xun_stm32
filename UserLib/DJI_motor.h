#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__


#include "stdint.h"
#include "can_bsp.h"
#include "pid.h"
#include "system.h"
#include "math.h"

typedef struct DJI_motor{
	int8_t id;						// 电机id
	int16_t __singel_angle;			// 转子当前单圈角度(8191->360度)
	int16_t __last_singel_angle;	// 转子上次单圈角度(8191->360度)
	int16_t __real_force;			// 转子实际转矩
	int16_t _now_speed;				// 转子当前转速(rpm)
	int32_t _retio;					// 减速比

	int32_t _all_angle;				// 角度总脉冲数( * 3.14 * 77.8 / 0x2000 / 36得到总数)

	int16_t set_force;				// 实时输出的转矩电流(-10000~10000 -> -10A~10A)
	int16_t real_speed;				// 轮子实际速度(mm/s)
	int16_t goal_speed;				// 轮子目标速度(mm/s)
	int16_t acc_speed;				// 经过加速度梯形曲线计算的轮子目标速度(mm/s)
	
	PID_t pid_speed;				// 速度闭环结构体
	
}DJI_motor;



void Dji_Init(void);
void Dji_recv(DJI_motor* motor, Can_Frame* frame);
void Dji_put_speed(void);
void Dji_motor_calc(float dt);
void Dji_reset_pos(void);


extern DJI_motor LU_Motor;		// id = 1
extern DJI_motor LD_Motor;		// id = 2		
extern DJI_motor RU_Motor;		// id = 3
extern DJI_motor RD_Motor;		// id = 4



#endif



