#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "system.h"
#include "arm.h"
//#include "car.h"


#define ARM_TASK_USE_STACK 512
#define CAR_TASK_USE_STACK 512

extern Arm RoboArm;

extern uint8_t arm_activate;

extern uint8_t paw_goal_state;
extern Emm_lock paw_now_state;




typedef enum ARM_ACTIVATE{
	ARM_DO_NOTHING = 0,	// 无事可做
	ARM_NEED_LOCK = 1,	// 以当前位置上锁
	ARM_NEED_UNLOCK,	// 解锁电机
	ARM_NEED_READ,		// 读取并显示当前位置
	ARM_NEED_SET,		// 设置为指定好了的位置
	ARM_PAW_LOCK,		// 爪子
	READ_PAW_STATE		//读取爪子状态
}ARM_ACTIVATE;

typedef enum CAR_ACTIVATE{
	CAR_DO_NOTHING = 0,	// 无事可做
	CAR_READ_PARAM = 1,	// 读取参数
	CAR_MOVE ,			//移动到指定位置
	CAR_KEEP_TURN,		//持续校正
	CAR_TURN,			//转向
	CAR_ADJUST			//校正
}CAR_ACTIVATE;



void Arm_Shell(int argc, char** argv);
void Arm_Task(void* param);
void Car_Shell(int argc, char** argv);
void Car_Task(void* param);


#endif



