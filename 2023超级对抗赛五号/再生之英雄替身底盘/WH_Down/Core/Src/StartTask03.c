#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "time_user.h"//不加这个也可以，因为里面是中断函数，系统会自己去找

void StartTask03(void const * argument)//??????????
{
 for (uint8_t i = 0; i < 4; i++)//???pid?????
	{
		pid_init(&motor_pid_can_2[i], 30, 0.5, 10, 16384, 16384); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
	}//P????????????????????????80??????z????????????100??????(??????????????>=320)

  for(;;)
  {
		for (uint8_t i = 0; i < 4; i++)
{			//?????482*19=9158
      motor_info_can_2[i].set_voltage = pid_calc(&motor_pid_can_2[i], target_speed_can_2[i], motor_info_can_2[i].rotor_speed);//????PID???
}//???????float???????int??????????????????????(????????????)
if(can_flag==1 && rc_ctrl.rc.s[1]==3)//can??u??,->3508
	{
		
			if((rc_ctrl.rc.ch[2]>=974&&rc_ctrl.rc.ch[2]<=1074)&&((rc_ctrl.rc.ch[3]>=974)&&(rc_ctrl.rc.ch[3]<=1074))&&(rc_ctrl.rc.ch[4]<=1074)&&(rc_ctrl.rc.ch[4]>=974))
		{
			for(int i=0;i<4;i++)//????
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
				sin_sita=(rc_ctrl.rc.ch[3]-1024)/r;
				cos_sita=(rc_ctrl.rc.ch[2]-1024)/r;
				target_v=(r/660)*9158;
				 
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
	

	}
else//????d???
	{
				motor_info_can_2[0].set_voltage=0;
				motor_info_can_2[1].set_voltage=0;
				motor_info_can_2[2].set_voltage=0;
				motor_info_can_2[3].set_voltage=0;
	}

		set_motor_voltage_can_2(0, 
                      motor_info_can_2[0].set_voltage, 
                      motor_info_can_2[1].set_voltage, 
                      motor_info_can_2[2].set_voltage, 
                      motor_info_can_2[3].set_voltage);
	
    osDelay(1);
  }
  
}
