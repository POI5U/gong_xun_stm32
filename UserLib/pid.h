#ifndef PID_H__
#define PID_H__



typedef enum pid_type{
	POS_PID,
	ADD_PID,
}PID_type;



// PID控制器结构体
typedef struct PID_t{
    float kp;				// 比例系数
    float ki;				// 积分系数
    float kd;				// 微分系数
    float i_add;			// 积分累加值
    float last_error;		// 上一次误差
	float i_add_max;		// 最大i输出值	(默认100)
	float out_max;			// 总最大输出限幅
	float out_min;			// 死区输出范围(最小输出值)
	float i_add_time;		// 积分达到最大的时间(ms)

	float last_output;		// 上一次的输出值(用于增量模式记录或者位置模式加速度使用)

	PID_type type;			// PID_H类型(增量式: ADD_PID 或者位置式: POS_PID)
} PID_t;




typedef struct pos_speed_control{
	float max_speed;
	float min_speed;
	float acc_up;
	float acc_down;
	float acc_up_percent;
	float acc_down_percent;
	float kp_rate_percent;
	float kp;
	
	float last_speed;
	
}pos_speed_control;





// 初始化PID控制器
void PID_Init(PID_t *pid, PID_type type, float kp, float ki, float kd,
	float out_max, float out_min, float i_add_time);


// 计算PID控制器输出
float PID_Calc(PID_t *pid, float error, float dt);

// 初始化位置闭环控制器
void Pos_speed_Init(pos_speed_control* pos_ring, float max_speed, float min_speed, float acc_up, float acc_down,
	float acc_up_percent, float acc_down_percent, float kp_rate_percent, float kp);

// 位置闭环计算输出
float Pos_speed_Calc(pos_speed_control* pos_ring, float goal_pos, float now_pos, float dt_ms);





#endif // PID_H__



