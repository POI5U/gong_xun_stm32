#include "system.h"
#include "arm.h"
#include "motor.h"
#include "path.h"
#include "DJI_motor.h"
#include "protocol.h"
#include "usart.h"


//#define USE_PROTOOL		// 是否使用野火调试工具



static uint8_t uart1_nowRx = 0;
static uint8_t uart2_nowRx = 0;
static uint8_t rx485_nowRx = 0;

// -------------------即将弃用
uint8_t Uart2_RecvBuffer[UART2_RECV_BUFFERSIZE] = { 0 };
uint32_t Uart2_RecvNum = 0;
// -------------------


uint8_t RS485_RecvBuffer[RS485_RECV_BUFFERSIZE] = { 0 };
uint32_t RS485_RecvNum = 0;

static enum LED_STATE PCB_LED = LED_GREEN;

Arm RoboArm = { 0 };





void usDelay(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim23, 0);  // 将计数器重置为0
    HAL_TIM_Base_Start(&htim23);        // 启动定时器
    while (__HAL_TIM_GET_COUNTER(&htim23) < us);  // 等待计数器达到指定值
    HAL_TIM_Base_Stop(&htim23);         // 停止定时器
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1){
		#ifdef USE_PROTOOL
		protocol_data_recv(&uart1_nowRx, 1);
		#else
		UART_Recv_Callback(uart1_nowRx);
		#endif
		
		HAL_UART_Receive_IT(&huart1, &uart1_nowRx, 1);
		
	}else if(huart == &huart2){
		RS485_RecvBuffer[RS485_RecvNum++] = rx485_nowRx;
		HAL_UART_Receive_IT(&huart2, &rx485_nowRx, 1);
		
	}
}


void print(const char* format, ...)
{
	static uint8_t Uart1_TxBuffer[UART1_SEND_BUFFERSIZE] = { 0 };
	va_list args;
	va_start(args, format);
	vsnprintf((char*)Uart1_TxBuffer,
			UART1_SEND_BUFFERSIZE, format, args);
	va_end(args);
	
//	while( HAL_UART_Transmit(&huart1, Uart1_TxBuffer,
//			strlen((char*)Uart1_TxBuffer), 1000) != HAL_OK);
	HAL_UART_Transmit(&huart1, Uart1_TxBuffer,
			strlen((char*)Uart1_TxBuffer), 1000);
	
}



void set_run_state(enum LED_STATE state)
{
	PCB_LED = state;
}



void System_Task(void* param)
{
	const TickType_t xFrequency = pdMS_TO_TICKS(30); // 100ms执行一次
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	protocol_init();
	uint8_t led_red = 0, led_green = 0, led_blue = 100;

	uint8_t led_tick = 1;
	while(1){
		
		led_red   = *((uint8_t*)&PCB_LED + 2);
		led_green = *((uint8_t*)&PCB_LED + 1);
		led_blue  = *((uint8_t*)&PCB_LED + 0);
		
		if ( led_tick) {
			WS2812_Ctrl(led_red, led_green, led_blue);
		}else{
			WS2812_Ctrl(0, 0, 0);
		}
		led_tick = !led_tick;
		
		//print("pos = %.2f                \r", LU_Motor._all_angle * 3.14 * 77.8 / 0x2000 / 36);
		
		#ifdef USE_PROTOOL
		receiving_process();
		int32_t angle = yaw;
		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &angle, 1);
		#endif
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}



void User_Hardware_Init(void)
{

	WS2812_Ctrl(25, 25, 25);
	HAL_Delay(1000);

	while(BMI088_init()){
		HAL_Delay(100);
	}
	HAL_TIM_Base_Start(&htim23);

	HAL_UART_Receive_IT(&huart1, &uart1_nowRx, 1);
	HAL_UART_Receive_IT(&huart2, &rx485_nowRx, 1);

	ShellCmdRegister("imu",
			"use\"imu yaw\" to get param yaw\r\n"
			"		also can read \"gyroz\", \"temp\"\r\n"
			"use\"imu recalc\" to re get gyroz error, and \"reset\" to zero the yaw\r\n",
			IMU_shell);

	ShellCmdRegister("arm",
			"normol cmd: \"lock\", \"unlock\", \"read\", \"speed\" \r\n"
			"use \"arm move {intU} {intL} {intR}\" to move to this position\r\n",
			Arm_Shell);

	ShellCmdRegister("car",
			"normol cmd: \"pos\", \"PU\", \"pid\", \"state\", \"move\" \r\n"
			"use \"car move {floatx} {floaty}\" to move to this position\r\n",
			Car_Shell);


	arm_init(&RoboArm, 1000000, 1000000, 1000000, 60);

	HAL_Delay(100);

	get_gz_error(10000);
	yaw = 0;
}



void creat_task(void)
{
	xTaskCreate(System_Task, "System_Task", SYSTEM_TASK_USE_STACK, NULL, 0, NULL);
	xTaskCreate(IMU_Task, "IMU_Task", IMU_TASK_USE_STACK, NULL, 5, NULL);
	xTaskCreate(Arm_Task, "Arm_Task", ARM_TASK_USE_STACK, NULL, 4, NULL);
	xTaskCreate(Car_Task, "Car_Task", CAR_TASK_USE_STACK, NULL, 4, NULL);
	xTaskCreate(Path_Task, "Path_Task", PATH_TASK_USE_STACK, NULL, 3, NULL);
	xTaskCreate(RS485_Task, "RS485_Task", RS485_TASK_USE_STACK, NULL, 5, NULL);
	xTaskCreate(RASPI_Task, "RASPI_Task", RASPI_TASK_USE_STACK, NULL, 5, NULL);
	
}



void Error_Handle(char* error_code)
{
	__disable_irq();
	
	while(1){
		print("error_code\r\n");
		
		volatile uint32_t delay_num = 275000000;	// 0.5s
		while( delay_num--);
		
	}
}
