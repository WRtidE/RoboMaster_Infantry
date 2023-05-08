#include "Yaw_task.h"
#include "motor.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "cmsis_os.h"


#define valve 100	
#define base 1024		
#define base_max 1684		
#define base_min 364
#define angle_valve 10	

fp32 ins_yaw;
fp32 ins_pitch;
fp32 ins_roll;
fp32 yaw_data;
fp32 init_yaw;	
fp32 err_yaw;		
fp32 angle_weight = 3;	
fp32 target_angle;
uint8_t reset_flag = 0;
uint8_t yaw_weight = 3;
infantry_control chassis;
ins_data_t ins_data;
pid_struct_t yaw_pid[7];	


extern int16_t Wz;
int yaw_model_flag = 1; 
extern UART_HandleTypeDef huart6;
extern uint8_t rx_yaw[100];

static void Yaw_init();	

static void Yaw_read_imu();

static void Yaw_mode_1();

static void Yaw_mode_2();

static void Yaw_calc_and_send();

static void chassis_follow();

static void auto_aim();

void gimbal_yaw_task(void const * argument)
{

	Yaw_init();
  
  for(;;)
  {
	  
	Yaw_read_imu();// ��ȡ��̨����������
	get_yaw_data();// ��ȡ��������������
	  
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
	pid_init(&motor_pid[4], 200 ,0, 0, 30000, 30000);//����ٶ�pid
	
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);//�����־λ
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); //ʹ��IDLE�ж�
	HAL_UART_Receive_DMA(&huart6,rx_yaw,100); //��������	

}

static void Yaw_read_imu()
{
	ins_yaw = ins_data.angle[0];
	ins_pitch = ins_data.angle[1];
	ins_roll = ins_data.angle[2];
}

static void Yaw_mode_1() //����̨ģʽ
{
	if(!mouse_x)
	{
		//ң�������У�Ĭ������̨
		if(rc_ctrl.rc.ch[0] > -50 && rc_ctrl.rc.ch[0] < 50)
		{
			if(yaw_model_flag == 1)	
			{
				init_yaw = ins_yaw;
				yaw_model_flag = 0;
			}

			err_yaw = ins_yaw  - init_yaw;
			
			if(err_yaw > 1 || err_yaw < -1)
			{
				target_speed[4] = err_yaw * angle_weight;   
			}
			else
			{
				target_speed[4] = 0;
			}
		}
		//����yaw�ᣬ��̨�ɶ�
		else if(rc_ctrl.rc.ch[0] > -660 &&rc_ctrl.rc.ch[0]<= 660)
		{
			target_speed[4] = -(rc_ctrl.rc.ch[0] / 660.00 * 100);
			yaw_model_flag = 1;
			
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
				target_speed[4] = err_yaw * angle_weight;
			}
			else
			{
				target_speed[4] = 0;
			}
		}
		//�ƶ���꣬��̨�ɶ�
		else if(mouse_x>=-16384&&mouse_x<=16384)
		{
			target_speed[4] = -((mouse_x ) / 16384.00 * 5000);
			yaw_model_flag = 1;
		}
		else if(mouse_x>16384)
		{
			mouse_x = 16384;
			target_speed[4] = -((mouse_x ) / 16384.00 * 5000);
			yaw_model_flag = 1;
		}
		else if(mouse_x<-16384)
		{
			mouse_x = -16384;
			target_speed[4] = -((mouse_x ) / 16384.00 * 5000);
			yaw_model_flag = 1;
		}
	}

	Yaw_calc_and_send();
    osDelay(1);
}

static void auto_aim()
{
	target_speed[4] = yaw_data * yaw_weight;
	Yaw_calc_and_send();
}


static void Yaw_calc_and_send()
{
	motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
	set_motor_voltage1(1, motor_info[4].set_voltage, 0, 0, 0);
}




//static void Yaw_mode_2()
//{
//	if (!mouse_x)
//	{
//		if (rc_ctrl.rc.ch[0] >= -50 && rc_ctrl.rc.ch[0] <= 50)
//		{
//		  if (motor_info[4].rotor_speed > 10 || motor_info[4].rotor_speed < -10) 
//		  {
//			target_speed[4] = 0;
//			motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
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
//			target_speed[4] = -((rc_ctrl.rc.ch[0] ) / 660.00 * 100);
//			motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
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
//			target_speed[4] = 0;
//			motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
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
//			target_speed[4] = -((mouse_x ) / 16384.00 * 5000);
//			yaw_model_flag = 1;
//		  }else if(mouse_x>16384){
//				mouse_x = 16384;
//				target_speed[4] = -((mouse_x ) / 16384.00 * 5000);
//				yaw_model_flag = 1;
//		  }else if(mouse_x<-16384){
//			  mouse_x = -16384;
//				target_speed[4] = -((mouse_x ) / 16384.00 * 5000);

//				yaw_model_flag = 1;
//		  }
//		}
//	}
//	Yaw_calc_and_send();
//    osDelay(1);
//}
