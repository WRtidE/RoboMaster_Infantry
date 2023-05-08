#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "time_user.h"//不加这个也可以，因为里面是中断函数，系统会自己去找




void StartTask02(void const * argument)//遥控器连接和初始化
	

{
  remote_control_init();
	motor_info[0].set_voltage=0;
	motor_info[1].set_voltage=0;
	motor_info[2].set_voltage=0;
	motor_info[3].set_voltage=0;//防止bug
  for(;;)
  {		
		if(rc_ctrl.rc.ch[3]>300)
		{
			can_flag=1;
		}
    osDelay(1);
  }

}
