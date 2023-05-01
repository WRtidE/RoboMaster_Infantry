#include "power_control.h"
#include "judge.h"
#include "math.h"

#define POWER_LIMIT         80.0f   //��������
#define POWER_BUFF_LIMIT    60.0f	//����������  
#define WARNING_POWER       40.0f   //���书��
#define WARNING_POWER_BUFF  50.0f   //�ٶ�һ��ͻᱬը

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, û�й��ʺ͵������Ƶ����
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f    //��������������
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f    //��������

/*
����ĵ���ֵ����24V�����̹����ѹ�����Ƶ��ڸõ���Ĺ���
���������������:����PID������ĸ�����ĵ���Ŀ��ֵ����������ڵ�ǰ���ù������ƣ�
��ͬ��������/��С�ٶȡ��ٴμ��㣬ֱ���ܹ���С������ֵ*/

void chassis_power_control()
{
    

	fp32 chassis_power 		  = infantry.chassis_power;  //���̹���
    fp32 chassis_power_buffer = infantry.chassis_power_buffer; //���̹��ʻ�����
    fp32 total_current_limit  = 0.0f;   //���������������
    fp32 total_current 		  = 0.0f;         //�����������
    
	if(infantry.Robot_id == 0)
	{
		 total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT; //���δ���Ӳ���ϵͳ���Ͳ�����������
	}
	else
	{
			//���ʳ���80w �ͻ�������С��60j,��Ϊ��������С��60��ζ�Ź��ʳ���80w
		if(chassis_power_buffer < WARNING_POWER_BUFF)   //��������С���趨ֵ��˵��������
		{
			fp32 power_scale;
			if(chassis_power_buffer > 5.0f)
			{
				//��СWARNING_POWER_BUFF
				power_scale = chassis_power_buffer / WARNING_POWER_BUFF;       //���������ʣ����>5.0������ = ��ǰ������ռ��ֵ/�ܻ�����ֵ
			}
			else
			{
				//only left 10% of WARNING_POWER_BUFF
				power_scale = 5.0f / WARNING_POWER_BUFF;      //�������������ʣ�� < 5.0f ,���ʮ�ֽ�����ǿ��������ԭ�ٶȵ�0.1
			}
			//scale down
			//��С
			total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;  //��ǰ�������� = �ܵ������� * ��С���� 
		}
		else
		{
			//power > WARNING_POWER
			//���ʴ���WARNING_POWER
			if(chassis_power > WARNING_POWER) //���� > ������Ԥ��
			{
				fp32 power_scale;

				//power < 80w
				//����С��80w��δ������
				if(chassis_power < POWER_LIMIT) //δ�����ʣ��������˾��书��
				{
					//scale down
					//��С
					power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
					
				}
				//power > 80w
				//���ʴ���80w��������
				else
				{
					power_scale = 0.0f; 
				}
				
				total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
			}
			//power < WARNING_POWER
			//����С��WARNING_POWER
			else
			{
				total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
			}
		}
	}


    
    total_current = 0.0f;
    //calculate the original motor current set
    //����ԭ����������趨
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(motor_pid[i].output); //���ظ������ľ���ֵ
    }
    

    if(total_current > total_current_limit) //����ܵ����趨�������ޣ��ͽ�������
    {
        fp32 current_scale = total_current_limit / total_current;
        motor_pid[0].output = current_scale;
		motor_pid[1].output = current_scale;
		motor_pid[2].output = current_scale;
		motor_pid[3].output = current_scale;
    }
}
