#include "Yaw_task.h"
#include "struct_typedef.h"
#include "cmsis_os.h"
#include "Chassis_task.h"
#include "drv_can.h"
#include "INS_task.h"

#define valve 100	
#define base 1024		
#define base_max 1684		
#define base_min 364
#define angle_valve 10	

fp32 ins_yaw;
fp32 ins_pitch;
fp32 ins_roll;
extern fp32 yaw_data;
fp32 init_yaw;	
fp32 err_yaw;
fp32 angle_weight = 3;	
uint8_t reset_flag = 0;
uint8_t yaw_weight = 3;



//volatile int16_t motor_speed_target[5];
extern  ins_data_t ins_data1;
extern  ins_data_t ins_data;
int8_t init_err = 0;
int8_t start_flag = 0;

fp32 chassis_yaw_pid [3]={200,0,0}; //yaw pid数组

extern pid_struct_t motor_pid_chassis[4];

extern int16_t Wz;
int yaw_model_flag = 1; 
 
extern UART_HandleTypeDef huart6;

static void Yaw_init();	

static void Yaw_read_imu();

static void Yaw_mode_1();

static void Yaw_mode_2();

static void Yaw_calc_and_send();

static void chassis_follow();

static void auto_aim();

static void start_check();


void gimbal_yaw_task(void const * argument)
{

	Yaw_init();
  
  for(;;)
  {
	  
	Yaw_read_imu();// 读取云台陀螺仪数据

	if(rc_ctrl.rc.s[1] == 1 && ins_yaw)
	{
		auto_aim();
	}
	else
	{
		Yaw_mode_1();
	}
	
    osDelay(1);
  }

}
 
static void Yaw_init()	
{
	pid_init(&motor_pid_chassis[4], chassis_yaw_pid, 30000, 30000);//电机速度pid
}

static void Yaw_read_imu() //insdata1是云台陀螺仪数据
{
	ins_yaw = ins_data1.angle[0];
	ins_pitch = ins_data1.angle[1];
	ins_roll = ins_data1.angle[2];
}

static void Yaw_mode_1() //锁云台模式
{
	if(!mouse_x)
	{
		//遥控器置中，默认锁云台
		if((rc_ctrl.rc.ch[0] > -50 && rc_ctrl.rc.ch[0] < 50) && ins_yaw )
		{
			if(yaw_model_flag == 1)	
			{
				init_yaw = ins_yaw;
				yaw_model_flag = 0;
			}

			err_yaw = ins_yaw  - init_yaw;
			
			if(err_yaw > 1 || err_yaw < -1)
			{
				motor_speed_target[4] = err_yaw * angle_weight;   
			}
			else
			{
				motor_speed_target[4] = 0;
			}
		}
		//拨动yaw轴，云台可动
		else if(rc_ctrl.rc.ch[0] > -660 &&rc_ctrl.rc.ch[0]<= 660)
		{
			motor_speed_target[4] = -(rc_ctrl.rc.ch[0] / 660.00 * 100);
			yaw_model_flag = 1;
			
		}
	}
	else
	{
		//鼠标置中，默认锁云台
		if(mouse_x >= -5 && mouse_x <= 5)
		{
			if(yaw_model_flag == 1)	
			{
				init_yaw = ins_yaw;
				yaw_model_flag = 0;
			}

			err_yaw = ins_yaw  - init_yaw;
			if(err_yaw > 1 || err_yaw < -1)
			{
				motor_speed_target[4] = err_yaw * angle_weight;
			}
			else
			{
				motor_speed_target[4] = 0;
			}
		}
		//移动鼠标，云台可动
		else if(mouse_x>=-16384&&mouse_x<=16384)
		{
			motor_speed_target[4] = -((mouse_x ) / 16384.00 * 5000);
			yaw_model_flag = 1;
		}
		else if(mouse_x>16384)
		{
			mouse_x = 16384;
			motor_speed_target[4] = -((mouse_x ) / 16384.00 * 5000);
			yaw_model_flag = 1;
		}
		else if(mouse_x<-16384)
		{
			mouse_x = -16384;
			motor_speed_target[4] = -((mouse_x ) / 16384.00 * 5000);
			yaw_model_flag = 1;
		}
	}

	Yaw_calc_and_send();
    osDelay(1);
}

static void auto_aim()
{
	motor_speed_target[4] = yaw_data * yaw_weight;
	Yaw_calc_and_send();
}


static void Yaw_calc_and_send()
{
	//motor_info[4].set_voltage = pid_calc(&motor_pid_chassis[4], motor_speed_target[4], motor_info[4].rotor_speed);
	motor_info_chassis[4].set_current = pid_calc(&motor_pid_chassis[4], motor_info_chassis[4].rotor_speed,motor_speed_target[4]);
	set_motor_voltage1(1, motor_info_chassis[4].set_current, 0, 0, 0);
}

static void start_check()
{
	while(!start_flag)
	{
		init_err = ins_data.angle[1] - ins_data1.angle[1];

		if(init_err > -10 && init_err<10)
		{
			start_flag = 1;
		}
	}
}

//static void Yaw_mode_2()
//{
//	if (!mouse_x)
//	{
//		if (rc_ctrl.rc.ch[0] >= -50 && rc_ctrl.rc.ch[0] <= 50)
//		{
//		  if (motor_info[4].rotor_speed > 10 || motor_info[4].rotor_speed < -10) 
//		  {
//			motor_speed_target[4] = 0;
//			motor_info[4].set_voltage = pid_calc(&motor_pid[4], motor_speed_target[4], motor_info[4].rotor_speed);
//			yaw_model_flag = 1;
//		  }
//		  else
//		  {
//			motor_info[4].set_voltage = 0;
//		  }
//		}
//		else
//		{
//		  if(rc_ctrl.rc.ch[0]>=-660&&rc_ctrl.rc.ch[0]<=660)
//		  {
//			motor_speed_target[4] = -((rc_ctrl.rc.ch[0] ) / 660.00 * 100);
//			motor_info[4].set_voltage = pid_calc(&motor_pid[4], motor_speed_target[4], motor_info[4].rotor_speed);
//			yaw_model_flag = 1;
//		  }
//		}
//	}
//	else
//	{
//		if (mouse_x >= -5 && mouse_x <= 5)
//		{
//		  if (motor_info[4].rotor_speed > 10 || motor_info[4].rotor_speed < -10) 
//		  {
//			motor_speed_target[4] = 0;
//			motor_info[4].set_voltage = pid_calc(&motor_pid[4], motor_speed_target[4], motor_info[4].rotor_speed);
//			yaw_model_flag = 1;
//		  }
//		  else
//		  {
//			motor_info[4].set_voltage = 0;
//		  }
//		}
//		else
//		{
//		  if(mouse_x>=-16384&&mouse_x<=16384)
//		  {
//			motor_speed_target[4] = -((mouse_x ) / 16384.00 * 5000);
//			yaw_model_flag = 1;
//		  }else if(mouse_x>16384){
//				mouse_x = 16384;
//				motor_speed_target[4] = -((mouse_x ) / 16384.00 * 5000);
//				yaw_model_flag = 1;
//		  }else if(mouse_x<-16384){
//			  mouse_x = -16384;
//				motor_speed_target[4] = -((mouse_x ) / 16384.00 * 5000);

//				yaw_model_flag = 1;
//		  }
//		}
//	}
//	Yaw_calc_and_send();
//    osDelay(1);
//}
