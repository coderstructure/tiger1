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

// PID ��ʼ������������������ֵ���޷���
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

// PID ���������ѹֵ
float pid_calc(pid_struct_t *pid , float ref, float fdb)
{
   pid->ref  = ref;
   pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;
  
  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max); // �����޷�
	
	pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max); // ����޷�
  return pid->output;

}	


//vofaʵʱ����

void  get_PIDdata(uint8_t *data, uint16_t size)
{
int startIdx,endIdx;		//������Ч���ݵ���ʼ�����ͽ�������
	char valueStr[10] = {0}; 	//������Ч���ݶ�Ӧ���ַ���
	float PIDpara;				//
	
	if(data[size-1] == '!')		//�����һλΪ�ַ�'!'(˵���£�==�����ж�ʱ�����˶���������ֵ��Ҳ���������Ϊ��ֵ(uint8_t�����ֵ)���Ҳ�Ҳ�����Ϊ��ֵ(�ַ�'!'��Ӧ��asciiֵ)
	{
		//�ҵ� '=' ������
		for(int i=0;i<size;i++)
		{
			if(data[i] == '=')
			{
				startIdx = i + 1;	//�ҵ���Ч������ʼ����
				break;
			}
		}
		//�ҵ� '!' ������
		for (int i = startIdx; i < size; i++)
        {
            if (data[i] == '!')
            {
                endIdx = i;		//�ҵ���Ч���ݽ�������
                break;
            }
        }
		//��ȡ '='��'!'֮�����ֵ
		if (startIdx > 0 && endIdx > startIdx)
		{
			strncpy(valueStr, (char*)&data[startIdx], endIdx - startIdx);	//����Ч���ݳ��ȵ��ַ���dataԴ�ַ����п�����valueStr�ַ�����
			valueStr[endIdx - startIdx] = '\0';	//��valueStr�ַ���β������'\0'����Ϊ�ַ���������־
			PIDpara = atof(valueStr);		//���ַ���ת��Ϊ������("2.32"-->2.32)
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
	
