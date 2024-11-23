#ifndef __IMU_H__
#define __IMU_H__


#include "BMI088driver.h"
#include "system.h"

#define IMU_TASK_USE_STACK 512



extern float gyroz, temp;
extern float yaw;


extern void IMU_Task(void* param);
extern void IMU_shell(int argc, char** argv);

#endif


