#include "IMU.h"


float gyroz, temp;
float yaw = 0;		// 不限制幅度


/**
 *	@brief: 积分yaw值函数, 为保证最佳效果, 必须在最高优先级
**/
void IMU_Task(void* param)
{
	const TickType_t xFrequency = pdMS_TO_TICKS(2); // 每2ms执行一次
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	while(1){
		gyroz = BMI088_read_gz();
		temp = BMI088_read_temp();
		yaw += gyroz * (float)0.002;
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}



void IMU_shell(int argc, char** argv)
{
	if( argc == 2) {
		if( strcmp(argv[1], "yaw") == 0){
			print("IMU param:yaw = %.2f\r\n", yaw);
			
		}else if( strcmp(argv[1], "temp") == 0){
			print("IMU param:temp = %.2f\r\n", temp);
			
		}else if( strcmp(argv[1], "gyroz") == 0){
			print("IMU param:gyroz = %.2f\r\n", gyroz);
			
		}else if( strcmp(argv[1], "recalc") == 0){
			print("IMU recalc gyroz start!\r\n");
			get_gz_error(10000);
			print("IMU recalc gyroz successful!\r\n");
			
		}else if( strcmp(argv[1], "reset") == 0){
			yaw = 0;
			print("IMU yaw reset successful! now yaw = %.2f!\r\n", yaw);
			
		}else{
			print("IMU have no param:%s!!\r\n", argv[1]);
		}
	}else{
		print("IMU param num err!!\r\n");
	}
	print(">>");
}

