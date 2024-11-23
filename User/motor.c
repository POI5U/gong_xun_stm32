#include "motor.h"
#include "arm.h"
#include "pid.h"
#include "Raspi.h"
#include "DJI_motor.h"
#include "CarLib.h"

static int32_t limit(int32_t input, int32_t max)
{
	if( input > max) {
		return max;
	}else if( input < -max) {
		return -max;
	}else{
		return input;
	}
}


PID_t pid_angle = { 0 };
float goal_angle = 0;

uint8_t arm_activate = 0;

uint8_t paw_goal_state = 0;
Emm_lock paw_now_state = 0;

void Arm_Shell(int argc, char** argv)
{
	if( argc == 2) {
		if( strcmp(argv[1], "lock") == 0) {
			arm_activate = ARM_NEED_LOCK;		// 需要上锁
		}else if( strcmp(argv[1], "unlock") == 0) {
			arm_activate = ARM_NEED_UNLOCK;		// 需要解锁
		}else if( strcmp(argv[1], "read") == 0) {
			arm_activate = ARM_NEED_READ;		// 不上锁或解锁, 只读取位置
		}else if( strcmp(argv[1], "speed") == 0) {
			print("arm speed%d, speedL:%d, speedR:%d.\r\n",
					RoboArm.U_motor.maxspeed,
					RoboArm.L_motor.maxspeed,
					RoboArm.R_motor.maxspeed );		// 读取位置
		}else{
			print("arm param error!\r\n");
		}

	}else if( argc == 3){
		
		char *endptr1;
		int32_t num1 = strtol(argv[2], &endptr1, 10);
		if( endptr1 == argv[2] || *endptr1 != '\0'){
				print("arm speed data error!\r\n");
		}
		
		if( strcmp(argv[1], "speedU") == 0) {
			RoboArm.U_motor.maxspeed = num1;
			print("arm motor U speed set success\r\n");
			
		}else if( strcmp(argv[1], "speedL") == 0) {
			RoboArm.L_motor.maxspeed = num1;
			print("arm motor L speed set success\r\n");
			
		}else if( strcmp(argv[1], "speedR") == 0) {
			RoboArm.R_motor.maxspeed = num1;
			print("arm motor R speed set success\r\n");
			
		}else if( strcmp(argv[1], "paw") == 0) {
			if( strcmp(argv[2], "open") == 0) {
				paw_goal_state = 0;
				arm_activate = ARM_PAW_LOCK;
				print("arm paw open\r\n");
			}else if( strcmp(argv[2], "close") == 0) {
				paw_goal_state = 1;
				arm_activate = ARM_PAW_LOCK;
				print("arm paw close\r\n");
			}else{
				print("arm paw error\r\n");
			}
			
		}else{
			print("arm param error!\r\n");
		}
		
	}else if( argc == 5) {
		if( strcmp(argv[1], "move") == 0) {
			char *endptr2, *endptr3, *endptr4;
			int32_t num2 = strtol(argv[2], &endptr2, 10);
			int32_t num3 = strtol(argv[3], &endptr3, 10);
			int32_t num4 = strtol(argv[4], &endptr4, 10);
			
			if ( (endptr2 == argv[2] || *endptr2 != '\0')	||
				 (endptr3 == argv[3] || *endptr3 != '\0')	||
				 (endptr4 == argv[4] || *endptr4 != '\0') )  {
				// 转换失败
				print("arm move input position error!!\r\n");
			} else {
				RoboArm.U_motor.goalPU = num2;
				RoboArm.L_motor.goalPU = num3;
				RoboArm.R_motor.goalPU = num4;
				arm_activate = ARM_NEED_SET;		// 设置为指定位置
				print("arm will move to U:%d, L:%d, R:%d\r\n", num2, num3, num4);
			}
		}
	}else {
		print("arm param num error!\r\n");
	}
}










void Arm_Task(void* param)
{
	const TickType_t xFrequency = pdMS_TO_TICKS(10); // 1ms执行一次
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	while(1) {
		while( arm_activate == 0 ){
			vTaskDelay(pdMS_TO_TICKS(1));
		}	// 等待需要执行命令
		
		switch( arm_activate) {
			case ARM_NEED_LOCK:
				Arm_lock(&RoboArm, ARM_LOCK);
				print("arm lock success,  %lld , %lld , %lld .\r\n",
				RoboArm.U_motor.nowPU, RoboArm.L_motor.nowPU, RoboArm.R_motor.nowPU);
			break;
			
			case ARM_NEED_UNLOCK:
				Arm_lock(&RoboArm, ARM_UNLOCK);
				print("arm unlock success!\r\n");
			break;
			
			case ARM_NEED_READ:
				arm_read_PU(&RoboArm);
//				print("arm read data U: %lld , L: %lld , R: %lld .\r\n", 
//				RoboArm.U_motor.nowPU, RoboArm.L_motor.nowPU, RoboArm.R_motor.nowPU);
			break;
			
			case ARM_NEED_SET:
				arm_set_PU(&RoboArm);
				print("arm set success!\r\n");
			break;
			
			case ARM_PAW_LOCK:
				paw_now_state = paw_set(&RoboArm.Paw, paw_goal_state);
				print("arm paw\r\n");
			break;
			
			case READ_PAW_STATE:
				paw_now_state = EMM_judge_pos(&RoboArm.Paw);
				print("read paw\r\n");
			break;
			
		}
//		print(">>");
		arm_activate = 0;
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}




void Car_Shell(int argc, char** argv)
{
	if( argc == 3) {
		if( strcmp(argv[1], "turn") == 0) {
			char *endptr2;
			float num2 = strtof(argv[2], &endptr2);
			
			if ( (endptr2 == argv[2] || *endptr2 != '\0')	 )  {
				// 转换失败
				print("car turn input error!!\r\n");
			} else {
				goal_angle = num2;
				Car_turn_set(goal_angle);
			}
		}else{
			print("car param num error!\r\n");
		}
	}else if( argc == 4) {
		if( strcmp(argv[1], "move") == 0) {
			float goal_x, goal_y;
			char *endptr2, *endptr3;
			float num2 = strtof(argv[2], &endptr2);
			float num3 = strtof(argv[3], &endptr3);
			
			if ( (endptr2 == argv[2] || *endptr2 != '\0')	||
				 (endptr3 == argv[3] || *endptr3 != '\0')	 )  {
				// 转换失败
				print("car move input position error!!\r\n");
			} else {
				goal_x = num2;
				goal_y = num3;
				Car_move_set(goal_x, goal_y);
				
				print("car will move to x:%.2f, y:%.2f\r\n", goal_x, goal_y);
			}
		}else{
			print("car param num error!\r\n");
		}
	}else {
		print("car param num error!\r\n");
	}
}








extern int8_t start_path;

void Car_Task(void* param)
{
	const int32_t tick_ms = 1;
	const TickType_t xFrequency = pdMS_TO_TICKS(tick_ms); // 1ms执行一次
	TickType_t xLastWakeTime = xTaskGetTickCount();


	can_bsp_init();
	Dji_Init();
	Carlib_Init(1);

	while(start_path == 0){
		vTaskDelay(10);
	}


	while(1) {
		
		Car_calc(yaw, tick_ms);
		
		Dji_motor_calc(tick_ms);
		
		Dji_put_speed();

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}

}


