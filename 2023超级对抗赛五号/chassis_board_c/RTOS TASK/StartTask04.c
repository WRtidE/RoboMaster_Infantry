#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "rc_potocal.h"
#include "PID.h"
#include "arm_math.h"
#include "StartTask04.h"

//	���������С����ģʽ�µĵ����˶�
//	ע�⣺����C��ĽǶȾ�����һ��


//����һЩȫ�ֱ���

uint16_t Up_ins_yaw; //��¼����һ��C�崫����yaw���ֵ
uint16_t Down_ins_yaw;	//��¼����һ��C���yaw���ֵ
fp32 Err_yaw;	//����C��֮���yaw��֮��Ĳ�ֵ���涨Ϊ�ϼ���
fp32 Err_accident = 0;	//����C���ڻ�е��װʱ���żȻ���
fp32 Down_ins_pitch;
fp32 Down_ins_row;
fp32 sin_a;		//��ֵ��sin��cos
fp32 cos_a;
fp32 sin_sita;
fp32 cos_sita;
//define for testing
uint16_t test_up_ins_yaw=180;

void StartTask04(void const * argument)
{

	//�ĸ����̵����ʼ��
	for (uint8_t i = 0; i < 4; i++)
	{
		pid_init(&motor_pid_can_2[i], 30, 0.5, 10, 16384, 16384); 
	}
	
  for(;;)
  {		
		
		//ģʽѡ��
		if(can_flag==1 && rc_ctrl.rc.s[1]==2)				
		{
			//��ȡ����C���YAW���ֵ
			//д����can���ն�����
		
		
			//������C��Ƕ�ֵ��ȡ
			Down_ins_yaw = ins_data.angle[0] + 180;
			Down_ins_pitch = ins_data.angle[1];
			Down_ins_row = ins_data.angle[2];
		
			
			//��ýǶȲ�ֵ������ɻ���
			Err_yaw = Up_ins_yaw - Down_ins_yaw;
			//Err_yaw = test_up_ins_yaw - Down_ins_yaw;
			Err_yaw = Err_yaw/57.3f;
			
			
			//��ò�ֵ��sin��cos
			cos_a = cos(Err_yaw);
			sin_a = sin(Err_yaw);
			
			
			//���̴���
			if((rc_ctrl.rc.ch[2]>=974&&rc_ctrl.rc.ch[2]<=1074)&&((rc_ctrl.rc.ch[3]>=974)&&(rc_ctrl.rc.ch[3]<=1074))&&(rc_ctrl.rc.ch[4]<=1074)&&(rc_ctrl.rc.ch[4]>=974))
			{
				for(int i=0;i<4;i++)
				{
					if(motor_info_can_2[i].rotor_speed>720||motor_info_can_2[i].rotor_speed<-720)
					{
						target_speed_can_2[i]=0;
						motor_info_can_2[i].set_voltage = pid_calc(&motor_pid_can_2[i], target_speed_can_2[i], motor_info_can_2[i].rotor_speed);
					}
					else
					{
						motor_info_can_2[i].set_voltage=0;
					}
				}
			}
			
			else
			{				
				if(rc_ctrl.rc.ch[4]>1074)
				{target_curl=sqrt((rc_ctrl.rc.ch[4]-1024)*(rc_ctrl.rc.ch[4]-1024))/660;}//??????????????????????????
				else if(rc_ctrl.rc.ch[4]<974)
				{target_curl=-(sqrt((rc_ctrl.rc.ch[4]-1024)*(rc_ctrl.rc.ch[4]-1024))/660);}
				else{target_curl=0;}
				target_curl=target_curl*9158;
				int16_t target_curl_int=target_curl;

				r=sqrt((rc_ctrl.rc.ch[3]-1024)*(rc_ctrl.rc.ch[3]-1024)+(rc_ctrl.rc.ch[2]-1024)*(rc_ctrl.rc.ch[2]-1024));
				sin_sita = (float)(rc_ctrl.rc.ch[3]-1024);
				cos_sita = (float)(rc_ctrl.rc.ch[2]-1024);
				target_v = (double)(9158/660);
				 
				if(target_curl==0)
					{	
					target_speed_can_2[3]=(0.707*target_v*(sin_sita*cos_a+cos_sita*sin_a-cos_sita*cos_a+sin_sita*sin_a))/2;
					target_speed_can_2[1]=-(0.707*target_v*(sin_sita*cos_a+cos_sita*sin_a-cos_sita*cos_a+sin_sita*sin_a))/2;
					target_speed_can_2[0]=(0.707*target_v*(sin_sita*cos_a+cos_sita*sin_a+cos_sita*cos_a-sin_a*sin_sita))/2;
					target_speed_can_2[2]=-(0.707*target_v*(sin_sita*cos_a+cos_sita*sin_a+cos_sita*cos_a-sin_a*sin_sita))/2;//???????????z???j??????????
					}
				else
					{
						//��������imu�������NAN������ǵ�DEBUGһ��--->�ѽ��2023.3.19
					target_int1=(0.707*(9158/660)*(sin_sita*cos_a+cos_sita*sin_a-cos_sita*cos_a+sin_sita*sin_a))/2;//??????double??float????????????????????????NAN,????????int?????????????'??uint??????????
					target_int2=(0.707*(9158/660)*(sin_sita*cos_a+cos_sita*sin_a-cos_sita*cos_a+sin_sita*sin_a))/2;

					target_speed_can_2[3]=(target_int1-target_curl_int)/2;
					target_speed_can_2[1]=(-target_int1-target_curl_int)/2;
					target_speed_can_2[0]=(target_int2-target_curl_int)/2;
					target_speed_can_2[2]=(-target_int2-target_curl_int)/2;
				}	
				}
			
			set_motor_voltage_can_2(0, 
                      motor_info_can_2[0].set_voltage, 
                      motor_info_can_2[1].set_voltage, 
                      motor_info_can_2[2].set_voltage, 
                      motor_info_can_2[3].set_voltage);
		}
	
		
		osDelay(1);
  }
}