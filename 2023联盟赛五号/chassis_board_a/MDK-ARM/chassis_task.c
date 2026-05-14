#include "chassis_task.h"
#include "motor.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "cmsis_os.h"
#include "math.h"
#include "power_control.h"
#include "Yaw_task.h"


#define angle_mid 702  //设置编码电机中值

int16_t err_angle = 0;
static void chassis_init();	
volatile int16_t Vx=0,Vy=0,Wz=0;
fp32 sin_a;		
fp32 cos_a;
int16_t Temp_Vx;
int16_t Temp_Vy;



// static void Yaw_choice();

void err_calc();
extern int16_t chassis_data;	
static void chassis_mode_1();
extern fp32 ins_yaw;

static void chassis_mode_2();

static void chassis_follow();

static void chassis_calc();

static void chassis_can_send();

static void chassis_motol_speed_calculate();

void chassis_task(void const * argument)
{
	chassis_init();
	
  for(;;)
  {
	
		if(rc_ctrl.rc.s[0] == 2 )
		{
			chassis_mode_1(); //小陀螺
 		}
		else
		{
			chassis_mode_2(); //正常调整
		}
    	osDelay(1);
  }
}


void chassis_init()
{
	for (uint8_t i = 0; i < 4; i++) 
  {
    pid_init(&motor_pid[i], 10, 0, 0, 3000, 3000); 
  }
	
}

void chassis_mode_2() //自由移动模式
{
	if(!w_flag && !s_flag && !a_flag && !d_flag && !q_flag && ! e_flag)
	{
		if( (rc_ctrl.rc.ch[2]>=-50&&rc_ctrl.rc.ch[2]<=50)&&((rc_ctrl.rc.ch[3]>=-50)&&(rc_ctrl.rc.ch[3]<=50))&&(rc_ctrl.rc.ch[4]<=50)&&(rc_ctrl.rc.ch[4]>=-50))
		{
			Vy=  0;
			Vx=  0;
			Wz=  0;
		
			for(int i=0;i<4;i++)//减速  slow_down
			{
			 
				if(motor_info[i].rotor_speed>360||motor_info[i].rotor_speed<-360)
				{
					motor_info[i].set_voltage = pid_calc(&motor_pid[i],  motor_info[i].rotor_speed, 0);
					
				}
				else
				{
					motor_info[i].set_voltage=0;
				}
				chassis_power_control();
				chassis_can_send();
			}
			if(rc_ctrl.rc.ch[0] > -50 && rc_ctrl.rc.ch[0] < 50)
			{
				chassis_follow();
			}
			chassis_power_control();
			chassis_can_send();
		  }
		  else
		  {
					Vy=  rc_ctrl.rc.ch[3]/660.0*8000;
					Vx=  rc_ctrl.rc.ch[2]/660.0*8000;
					Wz= -rc_ctrl.rc.ch[4]/660.0*8000;
			
		  }	
		
	}
	else
	{
		if(w_flag)
		{
			Vy = 1500;
		}
		else if(s_flag)
		{
			Vy = -1500;
		}
		else
		{
			Vy = 0;
		}
		if(d_flag)
		{
			Vx = 1500;
		}
		else if(a_flag)
		{
			Vx = -1500;
		}
		else
		{
			Vx = 0;
		}
		if(q_flag == 0x0040)
		{
			Wz = 2000;
		}
		else if(e_flag == 0x0080)
		{
			Wz = -2000;
		}
		else
		{
			Wz = 0;
		}
	}
	
	chassis_calc();
	for (uint8_t i = 0; i < 4; i++)
	{                                                                                                  
		motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed); 
	}   
	chassis_power_control();
	chassis_can_send();   
                                                                                  
}


void chassis_mode_1() //小陀螺模式
{
		

	if(!q_flag && !e_flag)
	{
		if(rc_ctrl.rc.ch[4] > 50 || rc_ctrl.rc.ch[4] < -50)  //操作手调整小陀螺
		{
			Wz = -rc_ctrl.rc.ch[4]/660.0*8000;
		} 
		else
		{
			Wz= 2000;
		}		
	}
	else
	{
		if(q_flag)
		{
			Wz = 2000;
		}
		else if(e_flag)         
		{
			Wz = -2000;
		}
		else
		{
			Wz = 2000 ; //设置小陀螺转速
		}
	}
	
	err_angle = ins_yaw - chassis_data;	
	err_angle = err_angle/57.3f;
	cos_a = cos(err_angle);
	sin_a = sin(err_angle);

		
	if( !w_flag && !s_flag && !a_flag && !d_flag)
	{
		
		Vy= rc_ctrl.rc.ch[3]/660.0*8000;
		Vx= rc_ctrl.rc.ch[2]/660.0*8000;
			
			
		//curl matrix * V
		Temp_Vx = Vx;
		Temp_Vy = Vy;
		Vx = Temp_Vx*cos_a - Temp_Vy*sin_a;
		Vy = Temp_Vx*sin_a + Temp_Vy*cos_a;

	}
				// moving control by keyboard
	else 
	{	
		if(w_flag)
		{
			Vy = 2000;
		}
		else if(s_flag)
		{
			Vy = -2000;
		}
		else
		{
			Vy = 0;
		}
		
		if(a_flag)
		{
			Vx = -2000;
		}
		else if(d_flag)
		{
			Vx = 2000;
		}	
		
		else 
		{
			Vx = 0;
		}
		
		//curl matrix * V
		Temp_Vx = Vx;
		Temp_Vy = Vy;
		Vx = - (Temp_Vx*cos_a - Temp_Vy*sin_a);
		Vy = - (Temp_Vx*sin_a + Temp_Vy*cos_a);
	}

	chassis_calc();
	for (uint8_t i = 0; i < 4; i++)
	{                                                                                                  
		motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed); 
	} 
	chassis_power_control();
	chassis_can_send();
}


void chassis_follow()
{
	err_angle = ins_yaw-chassis_data;
	
	if(rc_ctrl.rc.ch[0] >= - 50   & rc_ctrl.rc.ch[0] <= 50)
	{	
		err_calc();
		if(err_angle < -5 || err_angle >5)
		{
			Wz = err_angle * 30;
		}
		else
		{
			Wz = 0;
		}
	}
	
	
	chassis_calc();
	for (uint8_t i = 0; i < 4; i++)
	{                                                                                                  
		motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed); 
	} 
}

void chassis_calc()
{
	
	target_speed[CHAS_LF] =   Vx+Vy+Wz;   
    target_speed[CHAS_RF] =   Vx-Vy+Wz;    
    target_speed[CHAS_RB] =  -Vy-Vx+Wz;  
    target_speed[CHAS_LB] =   Vy-Vx+Wz; 
}

void err_calc()
{
		
	
	    if(err_angle > -10 && err_angle < 10 )
		{
			err_angle = 0;
		}
		else
		{
		if(err_angle > 0)
		{
			if(err_angle > 180)
			{
				err_angle -= 360;
			}		
		}
		else if (err_angle < 0 )
		{
			if(err_angle < -180)
			{
				err_angle += 360;
			}
		}
		}
	    	
}
	

void chassis_can_send()
{	  
	set_motor_voltage(0,
                      motor_info[0].set_voltage,
                      motor_info[1].set_voltage,
                      motor_info[2].set_voltage,
                      motor_info[3].set_voltage);
    osDelay(1);
}