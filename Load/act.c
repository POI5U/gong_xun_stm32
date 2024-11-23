#include "act.h"

#define mode_move_error 3
//#define arm_move_error 10000
#define max_size_thread 5
extern MiniCar Dcar;

Mode_Color mode_array[3] = {0};


static const double Rangle = 0.01745329252;	// ·ù¶È×ª½Ç¶È



void Car_move(float x_mm, float y_mm)
{
	
	Car_move_set(x_mm, y_mm);
	
	while( Car_check() != ANGLE_ONLY){
		vTaskDelay(10);
	}
	
	vTaskDelay(100);
}

void Car_adjust(Adjust adjust, int32_t x, int32_t y, float adjust_ratio)
{
	float adjust_x, adjust_y, nowx, nowy;
	
	nowx = Dcar.goal_x_mm;
	nowy = Dcar.goal_y_mm;
	
	if(adjust == SCAN_RING){
		adjust_y = (Scan_cross_array[1]-y)*adjust_ratio;
		adjust_x = (Scan_cross_array[0]-x)*adjust_ratio;
	}else if(adjust == SCAN_MODE){
		adjust_y = (Scan_weight_array[1]-y)*adjust_ratio;
		adjust_x = (Scan_weight_array[0]-x)*adjust_ratio;
	}else if(adjust == SCAN_MODE_ONCE){
		adjust_y = (Scan_weight_array[1]-y)*adjust_ratio;
		adjust_x = (Scan_weight_array[0]-x)*adjust_ratio;
	}

	double car_x =  + adjust_x * cos(yaw * Rangle) + adjust_y * sin(yaw * Rangle);
	double car_y =  - adjust_y * cos(yaw * Rangle) + adjust_x * sin(yaw * Rangle);
	
	Car_move_set(nowx + car_x, nowy + car_y);

	Dcar.goal_x_mm = nowx;
	Dcar.goal_y_mm = nowy;
	
	vTaskDelay(200);

}


void Car_turn(float goal_angle)
{
	Car_turn_set(goal_angle);
	while( fabs(goal_angle - yaw) > 0.5){
		vTaskDelay(10);
	}
	vTaskDelay(100);
}



void scan_qr(void)
{
	rec_sta = 0;
	uint8_t data_buf[3] = {0};
	Write_Data(QR_start, data_buf, 0);

	vTaskDelay(20);
	
	while(rec_sta != 1){
		vTaskDelay(100);
		print("dont\r\n");
			
	}
	rec_sta = 0;
	print("qr\r\n");
}

void arm_move(int64_t (*Motor_Encode)[5], uint32_t site_num)
{
	arm_activate = ARM_NEED_READ;
	vTaskDelay(20);
	for(int32_t i = 0; i<site_num; i++)
	{
		arm_lock(Motor_Encode[i][3]);
		RoboArm.U_motor.goalPU = Motor_Encode[i][0];
		RoboArm.L_motor.goalPU = Motor_Encode[i][1];
		RoboArm.R_motor.goalPU = Motor_Encode[i][2];
		vTaskDelay(100);

		arm_activate = ARM_NEED_SET;
		vTaskDelay(50);
		while( 	fabs(1.0*RoboArm.L_motor.goalPU - RoboArm.L_motor.nowPU) > Motor_Encode[i][4] ||
				fabs(1.0*RoboArm.R_motor.goalPU - RoboArm.R_motor.nowPU) > Motor_Encode[i][4] ||
				fabs(1.0*RoboArm.U_motor.goalPU - RoboArm.U_motor.nowPU) > Motor_Encode[i][4]  ){
			vTaskDelay(20);
			arm_activate = ARM_NEED_READ;
		}
	}
}

void arm_lock(uint8_t state){
	if(paw_goal_state != state){
		vTaskDelay(100);
		
		paw_goal_state = state;
		arm_activate = ARM_PAW_LOCK;
		print("paw\r\n");
		
		vTaskDelay(10);
		if(paw_now_state != state){
			print("p\r\n");
			vTaskDelay(20);
			arm_activate = ARM_PAW_LOCK;
		}
		
		vTaskDelay(100);
		
	}
}



void scan_adjust(Adjust adjust)
{
	while( 	fabs(1.0*RoboArm.L_motor.goalPU - RoboArm.L_motor.nowPU) > 100 ||
				fabs(1.0*RoboArm.R_motor.goalPU - RoboArm.R_motor.nowPU) > 100 ||
				fabs(1.0*RoboArm.U_motor.goalPU - RoboArm.U_motor.nowPU) > 100  ){
			vTaskDelay(30);
			arm_activate = ARM_NEED_READ;
		}
	rec_sta = 0;
	uint8_t data_buf[3] = {0};
	if(adjust == SCAN_RING){

		Write_Data(Ring_start, data_buf, 0);
		print("ring\r\n");
		
		while(rec_sta != 1){
		vTaskDelay(20);
		}
		rec_sta = 0;
		
	}else if(adjust == SCAN_MODE_ONCE){
		Write_Data(Weight_start, data_buf, 0);
		
		while(rec_sta != 1){
		vTaskDelay(20);
		}
		rec_sta = 0;
	}else if(adjust == SCAN_MODE){
		int8_t state = 0;
		int last_is_move = -1;
		int now_is_move = -1;
		
		for(uint8_t reset = 0; reset<2; reset++){
			mode_array[reset] = 0;
		}
		
		Write_Data(Weight_start, data_buf, 0);
		while(rec_sta != 1){
			vTaskDelay(50);
		}
		mode_array[0] = Scan_weight_array[2] == 0 ? 0:Scan_weight_array[2];
		do{
			last_is_move = now_is_move;
			
			int16_t last_x = Scan_weight_array[0];
			int16_t last_y = Scan_weight_array[1];
			
			rec_sta = 0;
			mode_state = 0;
			Write_Data(Weight_start, data_buf, 0);
			while(rec_sta != 1){
				vTaskDelay(50);
			}
			
			if(abs(last_x - Scan_weight_array[0])>mode_move_error  || 
				abs(last_y - Scan_weight_array[1])>mode_move_error ){
				if(state == 0){
					state += 2;
				}
				state++;
				state = state > max_size_thread ? max_size_thread : state;
			}else{
				if(state == 0){
					state -= 2;
				}
				state--;
				state = state < -max_size_thread ? -max_size_thread : state;
			}
			now_is_move = (state > 0 ? 1:0);

			print("%d %d %d\r\n",now_is_move,last_is_move,state);

			vTaskDelay(100);
			rec_sta = 0;
			
			if(mode_array[0] == 0 && Scan_weight_array[2] != 0){
				mode_array[0] = Scan_weight_array[2];
			}else if(mode_array[1] == 0 && Scan_weight_array[2] != 0 && Scan_weight_array[2] != mode_array[0]){
				mode_array[1] = Scan_weight_array[2];
			}
			
			if( now_is_move == 0 && last_is_move == 1 && mode_state == 1 && mode_array[1] != 0){ 
				break;
			}
		}while(1);
		mode_array[2] = (mode_array[0] != RED && mode_array[1] != RED) ? RED :
                        (mode_array[0] != GREEN && mode_array[1] != GREEN) ? GREEN : BLUE;
			
	}else{
		print("scan adjust error\r\n");
	}
	
}

void yaw_adjust(void){
	vTaskDelay(700);
	uint8_t data_buf[3] = {0};
	
	Write_Data(Yaw_Adjust_start, data_buf, 0);
	
	while(rec_sta != 1){
		vTaskDelay(20);
	}
	rec_sta = 0;
	vTaskDelay(800);
}



void arm_get_action(uint8_t num){			//ÔØÎïÌ¨×¥È¡ÅÐ¶Ï
	int8_t dnum = Scan_weight_array[2] -  QR_code_array[num];
	uint8_t mode_type = 0;
	for(uint8_t i = 0; i < 3; i++){
		uint8_t t = (i==2 ? 0:(i+1));
		if(mode_array[i] == RED && mode_array[t] == GREEN){
			mode_type = 1;
			break;
		}else if(mode_array[i] == RED && mode_array[t] == BLUE){
			mode_type = 2;
			break;
		}
	}
	if( dnum== 0){									//near
			arm_move(mid_get, 4);		
	}else if(mode_type == 1){
		if(dnum == 1 || dnum == -2){				//left
			arm_move(left_get, 4);	
		}else if(dnum == -1 || dnum == 2){			//right
			arm_move(right_get, 4);
		}
	}else if(mode_type == 2){
		if(dnum == 1 || dnum == -2){				//right
			arm_move(right_get, 4);			
		}else if(dnum == -1 || dnum == 2){			//left
			arm_move(left_get, 4);
		}
	}
}

void arm_p_put_action(uint8_t num){	//Ô²»·Î»ÖÃ·ÅÖÃÅÐ¶Ï
	int32_t u_speed, l_speed, r_speed, p_speed;
	u_speed = RoboArm.U_motor.maxspeed;
	l_speed = RoboArm.L_motor.maxspeed;
	r_speed = RoboArm.R_motor.maxspeed;
	p_speed = RoboArm.Paw.maxspeed;
	if( QR_code_array[num] == 1){					//ºì
		arm_move(p_right_put_before, 2);
		vTaskDelay(100);
		set_maxspeed(&RoboArm, 90000, 100000, 100000, 30);
		arm_move(p_right_put_after, 3);
	}else if(QR_code_array[num] == 2){				//ÂÌ	
		arm_move(p_mid_put_before, 2);
		vTaskDelay(100);
		set_maxspeed(&RoboArm, 90000, 100000, 100000, 30);
		arm_move(p_mid_put_after, 3);
	}else if(QR_code_array[num] == 3){				//À¶
		arm_move(p_left_put_before, 2);
		vTaskDelay(100);
		set_maxspeed(&RoboArm, 90000, 100000, 100000, 30);
		arm_move(p_left_put_after, 3);
	}
	set_maxspeed(&RoboArm, u_speed, l_speed, r_speed, p_speed);
}

void arm_p_put_on_mode_action(uint8_t num){	//Âë¶â·ÅÖÃÅÐ¶Ï
	if( QR_code_array[num] == 1){					//ºì
		arm_move(p_right_put, 4);
	}else if(QR_code_array[num] == 2){				//ÂÌ	
		arm_move(p_mid_put, 4);
	}else if(QR_code_array[num] == 3){				//À¶
		arm_move(p_left_put, 4);
	}
}

void arm_pd_get_action(uint8_t num){	//Ô²»·Î»ÖÃ×¥È¡ÅÐ¶Ï
	if( QR_code_array[num] == 1){					//ºì
		arm_move(pd_right_get, 5);
	}else if(QR_code_array[num] == 2){				//ÂÌ	
		arm_move(pd_mid_get, 5);
	}else if(QR_code_array[num] == 3){				//À¶
		arm_move(pd_left_get, 5);
	}
}

