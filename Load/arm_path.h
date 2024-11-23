#ifndef __LOAD1_H__
#define __LOAD1_H__

#include "stdint.h"




typedef enum Load_Site{
	CAR_GO,
	CAR_TURNN,
	CAR_SET,
	SCAN_QR,
	ARM_MOVE,
	ARM_LOCKK,
	SCAN_ADJUST,
	WAITING_STOP,
	ADD_NEW_SITE,
	
	
}Load_Site;

extern int64_t arm_move_start[2][5];
extern int64_t arm_move_state[1][5];
extern int64_t arm_look[1][5];
extern int64_t mid_get[4][5];
extern int64_t mid_put[4][5];
extern int64_t left_get[4][5];
extern int64_t left_put[4][5];
extern int64_t right_get[4][5];
extern int64_t right_put[4][5];

extern int64_t p_arm_look[1][5];
extern int64_t p_arm_high_look[1][5];
extern int64_t p_mid_get[4][5];
extern int64_t p_mid_put_before[2][5];
extern int64_t p_mid_put_after[3][5];
extern int64_t p_left_get[4][5];
extern int64_t p_left_put_before[2][5];
extern int64_t p_left_put_after[3][5];
extern int64_t p_right_get[4][5];
extern int64_t p_right_put_before[2][5];
extern int64_t p_right_put_after[3][5];

extern int64_t p_right_put[4][5];
extern int64_t p_left_put[4][5];
extern int64_t p_mid_put[4][5];


extern int64_t pd_mid_put[4][5];
extern int64_t pd_mid_get[5][5];
extern int64_t pd_left_put[4][5];
extern int64_t pd_left_get[5][5];
extern int64_t pd_right_put[4][5];
extern int64_t pd_right_get[5][5];





#endif


