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


fp32 ins_yaw;
fp32 ins_pitch;
fp32 ins_roll;
extern fp32 yaw_data;
fp32 init_yaw = 180;	
fp32 err_yaw;
fp32 err;
fp32 target_yaw;
//�ǶȻ�pid
fp32 angle_pid[3]={10,0,2.5};
pid_struct_t yaw_angle_pid[5];

fp32 angle_weight = 3;
uint8_t reset_flag = 0;
uint8_t yaw_weight = 3;



//volatile int16_t motor_speed_target[5];
extern  ins_data_t ins_data1;
extern  ins_data_t ins_data;
int8_t init_err = 0;
int8_t start_flag = 0;

fp32 chassis_yaw_pid [3]={300,0.1,0}; //yaw pid����


pid_struct_t motor_pid_yaw;

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

static void detel_calc();

void gimbal_yaw_task(void const * argument)
{

  Yaw_init();
  osDelay(10000);
  for(;;)
  {
	  
	Yaw_read_imu();// ��ȡ��̨����������

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
	pid_init(&motor_pid_yaw, chassis_yaw_pid, 30000, 30000);//����ٶ�pid
	pid_init(&yaw_angle_pid[4],angle_pid,30000,30000);      //�ǶȻ�pid
}

static void Yaw_read_imu() //insdata1����̨����������
{
	ins_yaw = ins_data1.angle[0];
	ins_pitch = ins_data1.angle[1];
	ins_roll = ins_data1.angle[2];
}

static void Yaw_mode_1() //����̨ģʽ
{
	if(!mouse_x)
	{

		
		if(rc_ctrl.rc.ch[0] >= -660 &&rc_ctrl.rc.ch[0]<= 660)
		{			
			init_yaw = init_yaw  + rc_ctrl.rc.ch[0]/660.0 * 0.5 + ((mouse_x ) / 16384.00 * 0.5); 
			
			detel_calc();
								
			motor_speed_target[4] =  - gimbal_PID_calc(&yaw_angle_pid[4], ins_yaw,init_yaw);
			
		}
		
	}
	else
	{
		//������У�Ĭ������̨
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
		//�ƶ���꣬��̨�ɶ�
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
	motor_info_chassis[4].set_current = pid_calc(&motor_pid_yaw, motor_info_chassis[4].rotor_speed,motor_speed_target[4]);
	set_motor_voltage1(1, motor_info_chassis[4].set_current, 0, 0, 0);
}

static void detel_calc()
{
	if(init_yaw >360)
	{
		init_yaw -=360;
	}
	
	else if(init_yaw<0)
	{
		init_yaw += 360;
	}
	
}


