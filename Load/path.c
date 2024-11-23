#include "path.h"
#include "motor.h"
#include "CarLib.h"
#include "act.h"

extern MiniCar Dcar;
int8_t start_path = 0;

void start(void){

	yaw = 0;
	
	start_path = 1;
}

void get_mode(uint8_t round)		//round是第几躺
{
	arm_move(arm_look, 1);
	vTaskDelay(100);
	scan_adjust(SCAN_MODE);
	
	Car_adjust(SCAN_MODE, 320, 200, 0.35);
	
	arm_get_action(3*(round-1));
	arm_move(mid_put, 4);
	arm_move(arm_look, 1);
	
	vTaskDelay(500);
	Scan_weight_array[2] = mode_array[2];
	arm_get_action(3*(round-1)+1);
	arm_move(left_put, 4);
	arm_move(arm_look, 1);

	arm_get_action(3*(round-1)+2);
	arm_move(right_put, 4);
	
	arm_move(arm_move_state, 1);
}

//on_mode用于是否码垛, 0为不码垛, 1为码垛
void put_to_ring(uint8_t round , uint8_t on_mode)
{
	arm_move(p_arm_high_look, 1);
	
	set_maxspeed(&RoboArm, 400000, 400000, 400000, 60);
	arm_move(p_arm_look, 1);
	set_maxspeed(&RoboArm, 1000000, 1000000, 1000000, 60);
	
	// 底盘减速, 便于调整
	float max_speed = Dcar.pos_close.max_speed;
	Dcar.pos_close.max_speed = 100;
	
	if(on_mode == 1){
		vTaskDelay(200);
		uint8_t buf[3] = {0};
		scan_adjust(SCAN_MODE_ONCE);
		
		Car_adjust(SCAN_MODE_ONCE, 320, 250, 0.428);

		vTaskDelay(1200);
		yaw_adjust();
		vTaskDelay(500);

		scan_adjust(SCAN_MODE_ONCE);
		Car_adjust(SCAN_MODE_ONCE, 320, 250, 0.28);

		vTaskDelay(500);
		scan_adjust(SCAN_MODE_ONCE);
		Car_adjust(SCAN_MODE_ONCE, 320, 250, 0.2);
	}else{
		scan_adjust(SCAN_RING);
		Car_adjust(SCAN_RING, 320, 250, 0.55);

		vTaskDelay(1400);
		yaw_adjust();

		
		vTaskDelay(500);
		scan_adjust(SCAN_RING);
		Car_adjust(SCAN_RING, 320, 250, 0.38);
		
		vTaskDelay(500);
		scan_adjust(SCAN_RING);
		Car_adjust(SCAN_RING, 320, 250, 0.2);
	}

	// 底盘恢复原版速度
	 Dcar.pos_close.max_speed = max_speed;
	
	arm_move(p_mid_get, 4);
	if(on_mode == 0){
		arm_p_put_action(3*(round-1));
	}else if(on_mode == 1){
		arm_p_put_on_mode_action(3*(round-1));
	}
	
	
	
	
	arm_move(p_left_get, 4);
	if(on_mode == 0){
		arm_p_put_action(3*(round-1) + 1);
	}else if(on_mode == 1){
		arm_p_put_on_mode_action(3*(round-1) + 1);
	}
	
	arm_move(p_right_get, 4);
	if(on_mode == 0){
		arm_p_put_action(3*(round-1) + 2);
	}else if(on_mode == 1){
		arm_p_put_on_mode_action(3*(round-1) + 2);
	}
}

void get_from_ring(uint8_t round)
{
	arm_pd_get_action(3*(round-1));
	arm_move(pd_mid_put, 4);
	
	arm_pd_get_action(3*(round-1)+1);
	arm_move(pd_left_put, 4);
	
	arm_pd_get_action(3*(round-1)+2);
	arm_move(pd_right_put, 4);
}


void Path_Task(void* param)
{
	arm_move(arm_move_start, 2);
	
	uint8_t buff[3] = { 0 };
	
	while(HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port,START_BUTTON_Pin) == GPIO_PIN_SET){
		vTaskDelay(30);
	}
	start();
	
	
	while(1) {
		vTaskDelay(30);
		Write_Data(Continue_start, buff, 0);
		if(rec_sta != 0){
			rec_sta = 0;
			break;
		}
	}
	rec_sta = 0;
	
	
	arm_move(arm_move_state, 1);
	Car_move(-200, 200);

	Car_move(-200, 660);

	scan_qr();

	Car_move(-165, 1400);	//圆盘位置
	Car_turn(-90);
	
	get_mode(1);		//圆盘抓取模型过程
	
	Car_move(-165, 975);
	Car_move(-1770, 975);	//粗加工区位置
	
	Car_turn(90);
	
	put_to_ring(1, 0);

	get_from_ring(1);
	
	Car_move(-1820, 1800);
	Car_move(-1045, 1800);		//精加工位置
	Car_turn(0);
	
	put_to_ring(1, 0);
	
	arm_move(arm_move_state, 1);
	
	Car_move(-140, 1800);
	Car_move(-140, 1430);	//转盘位置
	
	Car_set_xy(-165, 1400);	// 将当前累计位置设置为第一次时候的累计位置
	
	Car_turn(-90);
	
	
	get_mode(2);
	
	Car_move(-165, 975);
	Car_move(-1770, 975);	//粗加工区位置
	
	Car_turn(90);
	
	put_to_ring(2, 0);
	
	get_from_ring(2);
	
	Car_move(-1820, 1800);
	Car_move(-1045, 1800);		//精加工位置
	Car_turn(0);
	
	put_to_ring(2, 1);
	
	arm_move(arm_move_state, 1);

	Car_move(-155, 1830);
	Car_move(-150, 240);
	Car_move(40, 40);
	
	
	while(1){
		vTaskDelay(30);
	}
	
}
