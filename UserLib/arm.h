#ifndef __ARM_H__
#define __ARM_H__

#include "math.h"
#include "EMM_motor.h"
#include "LK_motor.h"




typedef enum Arm_Lock{
	ARM_UNLOCK = 0,
	ARM_LOCK = 1,
	
}Arm_Lock;

typedef struct Arm{
		LK_motor U_motor;
		LK_motor R_motor;
		LK_motor L_motor;
		EMM_motor Paw;
}Arm;


void Arm_lock(Arm* arm, Arm_Lock state);

void set_maxspeed(Arm* Arm, uint32_t Umaxspeed, uint32_t Lmaxspeed, uint32_t Rmaxspeed,uint16_t Pawspeed);

void arm_init(Arm* arm, uint32_t under_s, uint32_t left_s, uint32_t right_s, uint32_t paw_s);

uint8_t paw_set(EMM_motor* motor, uint8_t state);

void arm_read_PU(Arm* arm);
void arm_set_PU(Arm* arm);

#endif
