#include "arm.h"
//-----------

#define   PI   3.1415926f			
#define PI2AG(x)     (x/PI*180)	
#define   H    15.8f
#define   L1   14.8f
#define   L2   14.2f
#define   L3   12.0f

static void Arm_disable(Arm* arm)
{
	while( LK_motor_disable(&arm->U_motor) != HAL_OK) {
		vTaskDelay(1);
	}
	while( LK_motor_disable(&arm->L_motor) != HAL_OK) {
		vTaskDelay(1);
	}
	while( LK_motor_disable(&arm->R_motor) != HAL_OK) {
		vTaskDelay(1);
	}
	while( RS485_get_state() != HAL_OK) {
		vTaskDelay(1);
	}
	
}


void arm_read_PU(Arm* arm)
{
	while( LK_motor_read_angle(&arm->U_motor) != HAL_OK) {
		vTaskDelay(1);
	}
	while( LK_motor_read_angle(&arm->L_motor) != HAL_OK) {
		vTaskDelay(1);
	}
	while( LK_motor_read_angle(&arm->R_motor) != HAL_OK) {
		vTaskDelay(1);
	}
	while( RS485_get_state() != HAL_OK) {
		vTaskDelay(1);
	}
}

void arm_set_PU(Arm* arm)
{
	while( LK_Position_Control(&arm->R_motor) != HAL_OK){
		vTaskDelay(1);
	}
	while( LK_Position_Control(&arm->L_motor) != HAL_OK){
		vTaskDelay(1);
	}
	while( LK_Position_Control(&arm->U_motor) != HAL_OK){
		vTaskDelay(1);
	}
	while( RS485_get_state() != HAL_OK) {
		vTaskDelay(1);
	}
}



void Arm_lock(Arm* arm, Arm_Lock state)
{
	if(state == ARM_LOCK){
		arm_read_PU(arm);
		
		arm->R_motor.goalPU = arm->R_motor.nowPU;
		arm->L_motor.goalPU = arm->L_motor.nowPU;
		arm->U_motor.goalPU = arm->U_motor.nowPU;
		
		arm_set_PU(arm);
	}else{
		Arm_disable(arm);
	}
}


void set_maxspeed(Arm* Arm, uint32_t Umaxspeed,
	uint32_t Lmaxspeed, uint32_t Rmaxspeed, uint16_t Pawspeed)
{
	Arm->L_motor.maxspeed = Lmaxspeed;
	Arm->R_motor.maxspeed = Rmaxspeed;
	Arm->U_motor.maxspeed = Umaxspeed;
	Arm->Paw.maxspeed=Pawspeed;
}



void arm_init(Arm* arm, uint32_t under_s, uint32_t left_s, uint32_t right_s, uint32_t paw_s)
{
	arm->U_motor.id=1;
	arm->L_motor.id=2;
	arm->R_motor.id=3;
	arm->Paw.id=1;

	set_maxspeed(arm, under_s, left_s, right_s, paw_s);
}



uint8_t paw_set(EMM_motor* motor, uint8_t state)
{
	motor->state = state;
	while( EMM_set_position(motor) != HAL_OK) {
		vTaskDelay(1);
	}
	while( RS485_get_state() != HAL_OK) {
		vTaskDelay(1);
	}
	return motor->state;
}




