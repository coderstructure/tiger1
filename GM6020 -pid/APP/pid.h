#ifndef __PID_H__
#define __PID_H__

#include "math.h"
#include "stdint.h"

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

  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct_t;


void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
							
float pid_calc(pid_struct_t *pid , float ref, float fdb);
							
void  get_PIDdata(uint8_t *data, uint16_t size) ;

#endif														