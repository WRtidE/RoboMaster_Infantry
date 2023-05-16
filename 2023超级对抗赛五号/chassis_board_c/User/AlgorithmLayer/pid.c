
#include "pid.h"
#include "main.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
	
#define rad_format(Ang) loop_fp32_constrain((Ang), 0, 360)
	
	
//ѭ���޷�����
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}



void pid_init(pid_struct_t *pid,   fp32 PID[3], fp32 max_out, fp32 max_iout)   //pid�ṹ���ʼ���������������ֱ�����������ֵ���������ֵ
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
   
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}


fp32 pid_calc(pid_struct_t *pid, fp32 fdb, fp32 set)   //�ڶ�������Ϊ����ֵ������������ΪĿ��ֵ   
{
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
    pid->error[0] = set - fdb;

    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];

    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}

/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     gimbal_init:"gimbal_control"����ָ��.
  * @retval         none
  */
void gimbal_PID_init(gimbal_PID_t *pid,fp32 PID[3], fp32 maxout, fp32 max_iout)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = PID[0];
    pid->ki = PID[1];
    pid->kd = PID[2];

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

fp32 gimbal_PID_calc(pid_struct_t *pid, fp32 fdb, fp32 set)   //�ڶ�������Ϊ����ֵ������������ΪĿ��ֵ   
{
    fp32 err;
	pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
	err = set - fdb;
	
	if(err > 180)
	{
		err -= 360;
	}
	else if(err < -180)
	{
		err += 360;
	}
	
    pid->error[0] = err;

    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];

    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}
