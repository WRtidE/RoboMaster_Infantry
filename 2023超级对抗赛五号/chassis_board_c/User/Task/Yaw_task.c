#include "Yaw_task.h"
#include "struct_typedef.h"
#include "cmsis_os.h"
#include "Chassis_task.h"
#include "drv_can.h"
#include "INS_task.h"
#include "pid.h"

#define valve 100	
#define base 1024		
#define base_max 1684		
#define base_min 364
#define angle_valve 10	

//test
fp32 temp_c;

fp32 ins_yaw;
fp32 ins_pitch;
fp32 ins_roll;
fp32 ins_gyro;
extern fp32 yaw_data;
fp32 init_yaw = 180;	
fp32 err_yaw;
fp32 err;
fp32 target_yaw;
//角度环pid
fp32 angle_pid[3]={5,0,0};    //2,0,80
pid_struct_t yaw_angle_pid[5];
//自瞄速度环pid
pid_struct_t aim_speed_pid;
fp32 auto_speed_pid[3] = {100,0,0};



fp32 angle_weight = 3;
uint8_t reset_flag = 0;
uint8_t yaw_weight = 3;
//自瞄
fp32 aim_target;


//volatile int16_t motor_speed_target[5];
extern  ins_data_t ins_data1;  
extern  ins_data_t ins_data;
int8_t init_err = 0;
int8_t start_flag = 0;

fp32 chassis_yaw_pid [3]={300,0,10}; //yaw pid数组  100,40,0

pid_struct_t motor_pid_yaw;

extern int16_t Wz;
int yaw_model_flag = 1; 
 
extern UART_HandleTypeDef huart6;

static void Yaw_init();	

static void Yaw_read_imu();

static void Yaw_mode_1();

static void Yaw_calc_and_send();

static void auto_aim();

static void start_check();

static void detel_calc(fp32 *angle);

void gimbal_yaw_task(void const * argument)
{

  Yaw_init();
  osDelay(10000);
  for(;;)
  {
	  
	Yaw_read_imu();// 读取云台陀螺仪数据
	temp_c = (yaw_data/50.f - 30.f);

	if((rc_ctrl.rc.s[0]== 1 || press_right) && ins_yaw)
	{
		auto_aim();
	}
	else 
	{
		Yaw_mode_1();
	}
	
	  Yaw_calc_and_send();
    osDelay(1);
  }

}
 
static void Yaw_init()	
{
	pid_init(&motor_pid_yaw, chassis_yaw_pid, 15000, 15000);//电机速度pid
	pid_init(&yaw_angle_pid[4],angle_pid,15000,15000);      //角度环pid
	pid_init(&aim_speed_pid,auto_speed_pid,15000,15000);
	
}

static void Yaw_read_imu() //ins_data1是云台陀螺仪数据
{
	ins_yaw   = ins_data1.angle[0];
	ins_pitch = ins_data1.angle[1];
	ins_roll  = ins_data1.angle[2];
	ins_gyro  = -(ins_data1.gyro[2] / 100.f  - 30.f) * 19.8;
} 

static void Yaw_mode_1() //锁云台模式
{
	
		if(rc_ctrl.rc.ch[0] >= -660 &&rc_ctrl.rc.ch[0]<= 660)
		{			
			init_yaw = init_yaw  + rc_ctrl.rc.ch[0]/660.0 * 0.8 + ((mouse_x ) / 16384.00 * 20); 
			
			detel_calc(&init_yaw);
								
			err_yaw = init_yaw - ins_yaw;
			
			detel_calc(&err_yaw);
			
			//角度环PID
			if(err_yaw < -1 || err_yaw > 1)
			{
				motor_speed_target[4] =  - gimbal_PID_calc(&yaw_angle_pid[4], ins_yaw, init_yaw);
			}
			else
			{
				motor_speed_target[4] =  0;
			}
			

		}

    osDelay(1);
}
 
static void auto_aim()
{
	if(yaw_data)
	{
		init_yaw = ins_yaw  +  temp_c;
	}
	else
	{
		init_yaw = init_yaw  + rc_ctrl.rc.ch[0]/660.0 * 0.4 + ((mouse_x ) / 16384.00 * 10);
	}
	
	
	detel_calc(&init_yaw);
								
	motor_speed_target[4] =  - gimbal_PID_calc(&yaw_angle_pid[4], ins_yaw,init_yaw);
	
}


static void Yaw_calc_and_send()
{
	//速度环PID
	if((rc_ctrl.rc.s[0]== 1 || press_right) && ins_yaw)
	{
		motor_info_chassis[4].set_current = pid_calc(&aim_speed_pid, ins_gyro,motor_speed_target[4]);
	}
	else
	{
		motor_info_chassis[4].set_current = pid_calc(&motor_pid_yaw, ins_gyro,motor_speed_target[4]);
	}
	set_motor_voltage1(1, motor_info_chassis[4].set_current, 0, 0, 0);
}

static void detel_calc(fp32 *angle)
{
	if(*angle > 360)
	{
		*angle -=360;
	}
	
	else if(*angle<0)
	{
		*angle += 360;
	}
	
}


