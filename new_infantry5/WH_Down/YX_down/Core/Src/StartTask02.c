#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "time_user.h"//�������Ҳ���ԣ���Ϊ�������жϺ�����ϵͳ���Լ�ȥ��




void StartTask02(void const * argument)//ң�������Ӻͳ�ʼ��
	

{
  remote_control_init();
	motor_info[0].set_voltage=0;
	motor_info[1].set_voltage=0;
	motor_info[2].set_voltage=0;
	motor_info[3].set_voltage=0;//��ֹbug
  for(;;)
  {		
		if(rc_ctrl.rc.ch[3]>300)
		{
			can_flag=1;
		}
    osDelay(1);
  }

}
