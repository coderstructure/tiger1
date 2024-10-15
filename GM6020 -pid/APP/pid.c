#include "pid.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
extern pid_struct_t GM6020_pid;

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

// PID 初始化参数，输入输出最大值（限幅）
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max
              )
{
  pid->kp       = kp;
	 pid->ki      = ki;
  pid->kd       = kd;
  pid->i_max    = i_max;
  pid->out_max  = out_max;
	
}

// PID 计算输出电压值
float pid_calc(pid_struct_t *pid , float ref, float fdb)
{
   pid->ref  = ref;
   pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;
  
  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max); // 积分限幅
	
	pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max); // 输出限幅
  return pid->output;

}	


//vofa实时调参

void  get_PIDdata(uint8_t *data, uint16_t size)
{
int startIdx,endIdx;		//定义有效数据的起始索引和结束索引
	char valueStr[10] = {0}; 	//定义有效数据对应的字符串
	float PIDpara;				//
	
	if(data[size-1] == '!')		//当最后一位为字符'!'(说明下，==进行判断时，两端都必须是数值，也即左侧会解析为数值(uint8_t数组的值)，右侧也会解析为数值(字符'!'对应的ascii值)
	{
		//找到 '=' 的索引
		for(int i=0;i<size;i++)
		{
			if(data[i] == '=')
			{
				startIdx = i + 1;	//找到有效数据起始索引
				break;
			}
		}
		//找到 '!' 的索引
		for (int i = startIdx; i < size; i++)
        {
            if (data[i] == '!')
            {
                endIdx = i;		//找到有效数据结束索引
                break;
            }
        }
		//提取 '='与'!'之间的数值
		if (startIdx > 0 && endIdx > startIdx)
		{
			strncpy(valueStr, (char*)&data[startIdx], endIdx - startIdx);	//将有效数据长度的字符从data源字符串中拷贝到valueStr字符串中
			valueStr[endIdx - startIdx] = '\0';	//将valueStr字符串尾部补上'\0'，作为字符串结束标志
			PIDpara = atof(valueStr);		//将字符串转换为浮点数("2.32"-->2.32)
		}
   }
	
  if(data[0] == 'p' &&data[1] == 1)
	{
	    GM6020_pid.kp =PIDpara;
	}
	else if(data[0] == 'p' &&data[1] == 2)
	{
	    GM6020_pid.ki =PIDpara;
	}
	else if(data[0] == 'p' &&data[1] == 3)
	{
	    GM6020_pid.kd =PIDpara;
	}
		
	}
	
