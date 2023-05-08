#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "rc_potocal.h"
#include "PID.h"
#include "arm_math.h"
#include "StartTask04.h"

//	这个任务是小陀螺模式下的底盘运动
//	注意：上下C板的角度尽可能一样


//定义一些全局变量

uint16_t Up_ins_yaw; //记录上面一块C板传来的yaw轴的值
uint16_t Down_ins_yaw;	//记录下面一块C板的yaw轴的值
fp32 Err_yaw;	//上下C板之间的yaw轴之间的差值，规定为上减下
fp32 Err_accident = 0;	//上下C板在机械安装时候的偶然误差
fp32 Down_ins_pitch;
fp32 Down_ins_row;
fp32 sin_a;		//差值的sin和cos
fp32 cos_a;
fp32 sin_sita;
fp32 cos_sita;
//define for testing
uint16_t test_up_ins_yaw=180;

void StartTask04(void const * argument)
{

	//四个底盘电机初始化
	for (uint8_t i = 0; i < 4; i++)
	{
		pid_init(&motor_pid_can_2[i], 30, 0.5, 10, 16384, 16384); 
	}
	
  for(;;)
  {		
		
		//模式选择
		if(can_flag==1 && rc_ctrl.rc.s[1]==2)				
		{
			//获取上面C板的YAW轴的值
			//写在了can接收端里面
		
		
			//三个下C板角度值读取
			Down_ins_yaw = ins_data.angle[0] + 180;
			Down_ins_pitch = ins_data.angle[1];
			Down_ins_row = ins_data.angle[2];
		
			
			//获得角度差值并处理成弧度
			Err_yaw = Up_ins_yaw - Down_ins_yaw;
			//Err_yaw = test_up_ins_yaw - Down_ins_yaw;
			Err_yaw = Err_yaw/57.3f;
			
			
			//获得差值的sin和cos
			cos_a = cos(Err_yaw);
			sin_a = sin(Err_yaw);
			
			
			//底盘代码
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
						//这里会造成imu读数变成NAN，后面记得DEBUG一下--->已解决2023.3.19
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