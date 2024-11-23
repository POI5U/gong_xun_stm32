#ifndef __ACT_H__
#define __ACT_H__

#include "system.h"

#include "arm.h"
#include "math.h"
#include "motor.h"
#include "Raspi.h"
#include "CarLib.h"
#include "arm_path.h"

extern Arm RoboArm;

extern uint8_t arm_activate;
extern uint8_t car_activate;

/*
typedef struct Arm_Site{
	int64_t (*Motor_Encode)[5];
	uint32_t site_num;

}Arm_Site;
*/

typedef enum Adjust{
	SCAN_RING,
	SCAN_MODE,
	SCAN_MODE_ONCE
}Adjust;

typedef enum Mode_Color{
	NOCOLOR = 0,
	RED = 1,
	GREEN = 2,
	BLUE = 3
}Mode_Color;

extern Mode_Color mode_array[3];


void Car_turn(float goal_angle);
void Car_move(float x_mm, float y_mm);
void Car_adjust(Adjust adjust, int32_t x, int32_t y, float adjust_ratio);
void scan_qr(void);
void arm_move(int64_t (*Motor_Encode)[5], uint32_t site_num);
void arm_lock(uint8_t state);
void scan_adjust(Adjust adjust);
void arm_get_action(uint8_t num);
void arm_p_put_action(uint8_t num);
void arm_p_put_on_mode_action(uint8_t num);
void arm_pd_get_action(uint8_t num);
void yaw_adjust(void);

#endif


