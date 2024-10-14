#include "pid.h"
#include "string.h"
#include "main.h"
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

// 速度环PID 初始化参数，输入输出最大值（限幅）
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
							float i_max,
              float out_max)
{
  pid->kp    = kp;
	pid->ki     = ki;
  pid->kd      = kd;
	pid->i_max  = i_max;
  pid->out_max  = out_max;
	
}


//  位置pid计算得到反馈值
float location_pid_calc(pid_struct_t *pid , float ref, float fdb)
{
 
 	pid->ref  = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;
  if(pid->err[0]<0.5f&&pid->err[0]>-0.5f)  pid->err[0] = 0;  //死区设置
 
	pid->p_out_location  = pid->kp * pid->err[0];
  pid->i_out_location += pid->ki * pid->err[0];
  pid->d_out_location  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out_location, -pid->i_max, pid->i_max); // 积分限幅
	
	pid->output_location = pid->p_out_location + pid->i_out_location + pid->d_out_location;
  LIMIT_MIN_MAX(pid->output_location, -pid->out_max, pid->out_max); // 输出限幅
  return pid->output_location;

}	

float speed_pid_calc(pid_struct_t *pid , float ref, float fdb)
{
 
 	pid->ref  = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;
  if(pid->err[0]<0.5&&pid->err[0]>-0.5)  pid->err[0] = 0;  //死区设置
  pid->p_out_speed  = pid->kp * pid->err[0];
  pid->i_out_speed += pid->ki * pid->err[0];
  pid->d_out_speed  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out_speed, -pid->i_max, pid->i_max); // 积分限幅
	
	pid->output_speed = pid->p_out_speed + pid->i_out_speed + pid->d_out_speed;
  LIMIT_MIN_MAX(pid->output_speed, -pid->out_max, pid->out_max); // 输出限幅
  return pid->output_speed;

}	
