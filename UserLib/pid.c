#include "pid.h"
#include "system.h"

#define LIMIT(input, max)		\
do{								\
	if( input > max) {			\
		input = max;			\
	}else if( input < -max) {	\
		input = -max;			\
	}							\
}while(0)


#define OUTLIMIT(input, min)				\
do{											\
	if( input < min && input >= 0) {		\
		input = min;						\
	}else if( input > -min && input < 0) {	\
		input = -min;						\
	}										\
}while(0)



static float acc_change(float last, float now, float acc)
{
    float diff = now - last;
    if (fabs(diff) > acc) {
        return last + (diff > 0 ? acc : -acc);
    } else {
        return now;
    }
}



void PID_Init(PID_t *pid, PID_type type, float kp, float ki, float kd,
	float out_max, float out_min, float i_add_time)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->i_add = 0.0f;
	pid->last_error = 0.0f;
	pid->i_add_max = 100.0f;
	pid->out_max = out_max;
	pid->out_min = out_min;
	pid->i_add_time = i_add_time;
	pid->type = type;
	pid->last_output = 0.0f;	
}



/**
 * @brief 进行一次pid计算
 * @param pid: pid结构体
 * @param error: goal - real 被计算的error值
 * @param dt: 距离上次计算的时间(ms)
 */
float PID_Calc(PID_t *pid, float error, float dt)
{
	float i_change = dt * pid->i_add_max / pid->i_add_time;
	if( error > 0){
		pid->i_add += i_change;
	}else if ( error < 0){
		pid->i_add -= i_change;
	}
	
	LIMIT(pid->i_add, pid->i_add_max);

	float d_error = (error - pid->last_error) / dt;
	pid->last_error = error;

	float output =	pid->kp * error + 
					pid->ki * pid->i_add + 
					pid->kd * d_error;

	if( pid->type == ADD_PID) {		// 若为增量式
		output += pid->last_output;	// 则加上上次数值
	}
	
	OUTLIMIT(output, pid->out_min);
	LIMIT(output, pid->out_max);

	pid->last_output = output;
	return output;
}



void Pos_speed_Init(pos_speed_control* pos_ring, float max_speed, float min_speed, float acc_up, float acc_down,
	float acc_up_percent, float acc_down_percent, float kp_rate_percent, float kp)
{
	pos_ring->max_speed = max_speed;	// 最大输出速度
	pos_ring->min_speed = min_speed;	// 最大输出速度

	pos_ring->acc_up = acc_up / 1000;	// 加速度 mm/s -> mm/ms
	pos_ring->acc_down = acc_down / 1000;	// 减速度 mm/s -> mm/ms
	
	pos_ring->acc_up_percent = acc_up_percent;		// 加速路程占总路程的百分比
	pos_ring->acc_down_percent = acc_down_percent;	// 降速路程占总路程的百分比
	pos_ring->kp_rate_percent = kp_rate_percent;			// 微调路程占总路程的百分比
	pos_ring->kp = kp;		// 在目标点附近时微调位置kp
	pos_ring->last_speed = 0;
}

float Pos_speed_Calc(pos_speed_control* pos_ring, float goal_pos, float now_pos, float dt_ms)
{
	if( goal_pos == 0) {
		pos_ring->last_speed = 0;
		return 0;
	}
	
	float pos_rate = now_pos / goal_pos;	// 现在占总路程的比值
    float outspeed = 0.0f;

    if( pos_rate < pos_ring->acc_up_percent) {		
		// 加速阶段
		outspeed = pos_ring->last_speed + pos_ring->acc_up * dt_ms;

    } else if( pos_rate > pos_ring->acc_up_percent && 
		pos_rate < pos_ring->acc_down_percent) {
        // 匀速阶段
		outspeed = pos_ring->last_speed;

    } else if( pos_rate > pos_ring->acc_down_percent && 
		pos_rate < pos_ring->kp_rate_percent) {
		// 减速阶段
		outspeed = pos_ring->last_speed - pos_ring->acc_down * dt_ms;

    } else {
        // 微调阶段
        outspeed = pos_ring->kp * ( goal_pos - now_pos);
    }
	
    LIMIT(outspeed, pos_ring->max_speed);
	OUTLIMIT(outspeed, pos_ring->min_speed);
	
	pos_ring->last_speed = outspeed;
    return outspeed;
}



