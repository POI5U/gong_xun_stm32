#ifndef __EMM_MOTOR_H__
#define __EMM_MOTOR_H__

#include "motor_midware.h"


#define EMM_LOCK_POS 670
#define EMM_UNLOCK_POS 300

typedef enum Emm_lock{
	PAW_UNLOCK = 0,
	PAW_LOCK = 1,
	
}Emm_lock;


typedef struct EMM_motor{
	uint8_t id;
	int16_t maxspeed;
	uint8_t state;
	uint32_t position;
	uint8_t callback;
}EMM_motor;


HAL_StatusTypeDef EMM_set_position(EMM_motor* motor);		
HAL_StatusTypeDef EMM_read_position(EMM_motor* motor);
HAL_StatusTypeDef EMM_motor_enable(EMM_motor* motor,Emm_lock State);
Emm_lock EMM_judge_pos(EMM_motor* motor);



#endif
