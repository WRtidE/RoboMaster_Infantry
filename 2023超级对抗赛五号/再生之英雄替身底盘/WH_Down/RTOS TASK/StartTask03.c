#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "rc_potocal.h"
#include "PID.h"
#include "arm_math.h"
#include "StartTask03.h"

// 这是底盘的第一代代码，将直接在这个基础上修改为云台跟随底盘（遥控器）
//	思路：在移动右摇杆时屏蔽跟随系统，在回中时以云台主控和底盘主控的陀螺仪差值角度环进行纠正（底盘转动），同时云台主控锁云台来抵消
//				底盘旋转带来的云台转动

extern fp32 sin_sita;
extern fp32 cos_sita;
extern RC_ctrl_t rc_ctrl;
void StartTask03(void const * argument)
{
	

	//Init PID data for 4 wheels
  for (uint8_t i = 0; i < 4; i++)
	{
		pid_init(&motor_pid_can_2[i], 30, 0.5, 10, 16384, 16384); 
	}

	
	//Major cycle
  for(;;)
  {
		for (uint8_t i = 0; i < 4; i++)
{			
      motor_info_can_2[i].set_voltage = pid_calc(&motor_pid_can_2[i], target_speed_can_2[i], motor_info_can_2[i].rotor_speed);//????PID???
}
if(can_flag==1 && rc_ctrl.rc.s[1]==3)
	{
		
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

				//r=sqrt((rc_ctrl.rc.ch[3]-1024)*(rc_ctrl.rc.ch[3]-1024)+(rc_ctrl.rc.ch[2]-1024)*(rc_ctrl.rc.ch[2]-1024));
				sin_sita = (float)(rc_ctrl.rc.ch[3]-1024);
				cos_sita = (float)(rc_ctrl.rc.ch[2]-1024);
				target_v = (double)(9158/660);
				 
				if(target_curl==0)
				{
					target_speed_can_2[3]=(0.707*target_v*(sin_sita-cos_sita))/2;
					target_speed_can_2[1]=-(0.707*target_v*(sin_sita-cos_sita))/2;
					target_speed_can_2[0]=(0.707*target_v*(sin_sita+cos_sita))/2;
					target_speed_can_2[2]=-(0.707*target_v*(sin_sita+cos_sita))/2;//???????????z???j??????????
				}
				else
				{
					target_int1=(0.707*target_v*(sin_sita-cos_sita))/2;//??????double??float????????????????????????NAN,????????int?????????????'??uint??????????
					target_int2=(0.707*target_v*(sin_sita+cos_sita))/2;

					target_speed_can_2[3]=(target_int1-target_curl_int)/2;
					target_speed_can_2[1]=(-target_int1-target_curl_int)/2;
					target_speed_can_2[0]=(target_int2-target_curl_int)/2;
					target_speed_can_2[2]=(-target_int2-target_curl_int)/2;//????2?????
				}
		}
				set_motor_voltage_can_2(0, 
                      motor_info_can_2[0].set_voltage, 
                      motor_info_can_2[1].set_voltage, 
                      motor_info_can_2[2].set_voltage, 
                      motor_info_can_2[3].set_voltage);
	

	}
//else
//	{
//				motor_info_can_2[0].set_voltage=0;
//				motor_info_can_2[1].set_voltage=0;
//				motor_info_can_2[2].set_voltage=0;
//				motor_info_can_2[3].set_voltage=0;
//	}
	
    osDelay(1);
  }
  
}
