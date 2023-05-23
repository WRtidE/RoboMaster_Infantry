#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "rc_potocal.h"
#include "PID.h"
#include "arm_math.h"
#include "super_cap.h"
#include  "drv_can.h"
#include  "drv_usart.h"
#include "judge.h"
#include "Chassis_task.h"

extern int flag[1];
extern float powerdata[4];

void super_cap(void const *pvParameters)//³¬µç
{
	for(;;)
		{  
	if(infantry.Robot_id==1)
	{
		supercap_uart_send(60);
	}
	if(infantry.Robot_id==2)
	{
		supercap_uart_send(80);
	}
	if(infantry.Robot_id==3)
	{
		supercap_uart_send(100);
	}
	else
	{

		supercap_uart_send(50);
	}

			osDelay(1);
		}

}