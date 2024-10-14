#ifndef __PID_H__
#define __PID_H__

#include "math.h"


typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  
  
	float i_max;
  float out_max;
 
  
  float ref;      // 目标速度
  float fdb;      // 反馈速度
  float err[2];   // 本次误差和上次误差

  float p_out_speed;
  float i_out_speed;
  float d_out_speed;
  float output_speed;

  float p_out_location;
  float i_out_location;
  float d_out_location;
  float output_location;
	
	int cnt;
}pid_struct_t;


void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
							float i_max,
              float out_max);
							
float location_pid_calc(pid_struct_t *pid , float ref, float fdb);
							
float speed_pid_calc(pid_struct_t *pid , float ref, float fdb);
							
//double msp(double x, double in_min, double in_max, double out_min, double out_max);
							
#endif										
							