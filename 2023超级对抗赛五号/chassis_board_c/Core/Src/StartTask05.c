#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "time_user.h"//²»¼ÓÕâ¸öÒ²¿ÉÒÔ£¬ÒòÎªÀïÃæÊÇÖĞ¶Ïº¯Êı£¬ÏµÍ³»á×Ô¼ºÈ¥ÕÒ

void StartTask05(void const * argument)		//???????g?
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
	uint16_t last_yaw_angel=motor_info[5].rotor_angle;
	uint16_t err_yaw_angel;
	float cmpensate_yaw;//?????
	for (uint8_t i = 0; i < 4; i++)//???pid?????
	{
		pid_init(&motor_pid_can_2[i], 30, 0.5, 10, 16384, 16384); 
	}
	pid_init(&motor_pid[5],60,10,5,30000,30000);//??'??????????pid
	
  for(;;)
  {
		//?????????g?
		if(rc_ctrl.rc.s[1]==2 && can_flag==1)
		{
			err_yaw_angel=motor_info[5].rotor_angle - last_yaw_angel;	//??????????j???????
			
			//????????
			if(rc_ctrl.rc.ch[4]>1074)
			{target_curl=sqrt((rc_ctrl.rc.ch[4]-1024)*(rc_ctrl.rc.ch[4]-1024))/660;}
			else if(rc_ctrl.rc.ch[4]<974)
			{target_curl=-(sqrt((rc_ctrl.rc.ch[4]-1024)*(rc_ctrl.rc.ch[4]-1024))/660);}
			else{target_curl=0;}
			//????????target_curl?????????h??0-1?i??????
			target_curl=target_curl*9158;
			int16_t target_curl_int=target_curl;
			
			//?????????????????
					target_speed_can_2[3]=target_curl_int;
					target_speed_can_2[1]=target_curl_int;
					target_speed_can_2[0]=target_curl_int;
					target_speed_can_2[2]=target_curl_int;
			
			//??????????????????????PID??
					for (uint8_t i = 0; i < 4; i++)
					{					
						motor_info_can_2[i].set_voltage = pid_calc(&motor_pid_can_2[i], target_speed_can_2[i], motor_info_can_2[i].rotor_speed);//????PID???
					}
					
			//??????????can?????
					set_motor_voltage_can_2(0, 
                      motor_info_can_2[0].set_voltage, 
                      motor_info_can_2[1].set_voltage, 
                      motor_info_can_2[2].set_voltage, 
                      motor_info_can_2[3].set_voltage);
					
			//?????????????????????????????yaw??????? 3
			cmpensate_yaw=-tyro*((target_curl/19)/yaw_L)*0.45f ;
			
			//????yaw??PID??????
			motor_info[5].set_voltage = pid_calc(&motor_pid[5], cmpensate_yaw, motor_info[5].rotor_speed);
			set_motor_voltage(1, 
                      motor_info[4].set_voltage, 
                      motor_info[5].set_voltage, 
                      motor_info[6].set_voltage, 
                      0);
			
			last_yaw_angel=motor_info[5].rotor_angle;	//??¼??h?e??
		}
		
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
}