#ifndef __LK_MOTOR_H__
#define __LK_MOTOR_H__
#include "motor_midware.h"



typedef struct LK_motor{
	uint8_t id;
	int32_t maxspeed;
	int64_t goalPU;
	int64_t nowPU;
	
}LK_motor;

typedef enum LK_lock{
	MOTOR_UNLOCK = 0,
	MOTOR_LOCK = 1,
	
}LK_lock;



HAL_StatusTypeDef LK_motor_disable(LK_motor* motor);
HAL_StatusTypeDef LK_motor_read_angle(LK_motor* motor);
HAL_StatusTypeDef LK_Position_Control (LK_motor* motor);



#endif

