#include "fix_yaw.h"

//	˼·�������ٶȻ�������PID��������̨������̨�����ǵ�yawֵ�������ǶȻ����в���
//  ע�⣺��̨����ת�Ǹ���
//  ��һ�汾�����ң�������Ƶİ汾

//	����һЩȫ�ֱ���

fp32 ins_yaw;
fp32 ins_pitch;
fp32 ins_row;
fp32 init_yaw;	//��סinit_yaw��ֵ
int yaw_model_flag = 1;
fp32 err_yaw;
fp32 angle_weight = 1;	//�ǶȻ�->�ٶȻ���ӳ���Ȩ��

#define valve 100		//��ֵ
#define base 1024		//ң�����Ļ���ֵ
#define base_max 1684		
#define base_min 364
#define angle_valve 10		//�Ƕ���ֵ���������Χ�ھͲ�ȥ������

void FIX_Task(void const *pvParameters)
{
  //��ʼ������
	
	//yaw������ʼ����idΪcan1��5��
	pid_init(&motor_pid[5],60,10,5,30000,30000);
	
	
	//ѭ����������
  for(;;)
  {
		
		//�����Ƕ�ֵ��ȡ
		ins_yaw = ins_data.angle[0];
		ins_pitch = ins_data.angle[1];
		ins_row = ins_data.angle[2];
		
		//ģʽ�ж�,���Ͻǿ��ؿ������·�
		if(rc_ctrl.rc.s[1] == 2 && ins_yaw)
		{
			
		
				//ң�л�������̨
				if(rc_ctrl.rc.ch[0] > base-valve && rc_ctrl.rc.ch[0] < base+valve)
				{
					
					
					
					if(yaw_model_flag == 1)		//�ƶ���̨�����¼�ס��̨�ĳ�ʼλ�õ�ֵ
					{
						init_yaw = ins_yaw;
						yaw_model_flag = 0;
					}
					
					
					err_yaw = ins_yaw - init_yaw;		//��ʵʱ���ݼ���ʼ����
					
					//��ֵ�ж�
					if(err_yaw > angle_valve || err_yaw < -angle_valve)
					{
						target_speed[5] = err_yaw * angle_weight;
					}
					
					else
					{
						target_speed[5] = 0;
					}

				}
				
				
				//�����е�ʱ������ƶ���̨
				else if(rc_ctrl.rc.ch[0] >= base+valve && rc_ctrl.rc.ch[0] <= base_max)
				{
					target_speed[5] = 60;
					yaw_model_flag = 1;
				}
				else if(rc_ctrl.rc.ch[0] >= base_min && rc_ctrl.rc.ch[0]<base - valve)
				{
					target_speed[5] = -60;
					yaw_model_flag = 1;
				}
				
				//����pid�����ͺ�������������������
				motor_info[5].set_voltage = pid_calc(&motor_pid[5], target_speed[5], motor_info[5].rotor_speed);
			
				
		}
		
    osDelay(1);
  }

}