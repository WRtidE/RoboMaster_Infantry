#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "Can_user.h"
#include "Friction_task.h"
#include "Exchange_task.h"
#include "struct_typedef.h"

fp32 target_angle1;
uint8_t r_model= 0; //�жϵ��ָ�ģʽ 0��ֹ 1�� 2�ر�
uint8_t friction_flag = 0;

fp32 count = 0;
fp32 heat_warning = 0;
fp32 shoot_speed_set;
fp32 speed_choice;



//PID��ʼ��
static void Friction_init();

//ģʽѡ��
static void model_choice();



//����
static void magazine_task();
static void magazine_init();

//ǹ����������
static void heat_limit();

//can����
static void shoot_data_send();

//���⼤��ʹ��
static void laser_init();

static void friction_enable();
static void friction_disable();

static void friction_flag_change();

//����
static void trigger_task();

//���ݲ���ϵͳ�ı䵯��
static void shoot_speed_choice();


void Friction_task(void const * argument)
{
  Friction_init();
  magazine_init();
  laser_init();
		
  for(;;)
  {	  
	shoot_speed_choice();
	model_choice();
	magazine_task();
	heat_limit();       
    shoot_data_send();	  
  }
  osDelay(1);
}

static void Friction_init()
{
  pid_init(&motor_pid[0], 30,   0, 0, 16384, 16384); 
	pid_init(&motor_pid[1], 30,   0, 0, 16384, 16384);
	pid_init(&motor_pid[2], 20 ,   0, 0, 16384, 16384);	
}

static void laser_init()
{
	HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_SET);
}

//===========================================ǹ�ڵ���ѡ��===========================================
static void shoot_speed_choice()
{
	
	if(infantry.speed_limit == 15 ) //��������
	{
		shoot_speed_set = 3950;
	}
	else if(infantry.speed_limit == 18 ) //��ȴ����
	{
		shoot_speed_set = 4625;  
	}
	else if(infantry.speed_limit == 30) //��������
	{
		shoot_speed_set = 10000;
	}
	else
	{
		shoot_speed_set = 8700;
	}
		
}

//===========================================����Ħ����============================================================
static void model_choice()
{
	friction_flag_change(); //��ȡ�����Ƿ���
	if(friction_flag||rc_ctrl.rc.s[1] == 1)
	{
		friction_enable();
	}
	else //����������ת����0
	{
		friction_disable();
	}
	trigger_task();
}

static void friction_enable()
{

		target_speed[0] = -shoot_speed_set;//-8700
        target_speed[1] =  shoot_speed_set;// 8700
	
		motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
        motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
	
}

static void friction_disable()
{
	
	  target_speed[0] = 0;
      target_speed[1] = 0;
	  
		
	  motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
      motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
	
	
}

static void friction_flag_change()
{
	if(q_flag)
	{
		friction_flag = friction_flag + 1;
	}
	else if(e_flag)
	{
		friction_flag = 0;
	}
}
//=============================================���̵��==========================================================
static void trigger_task()
{
	if(rc_ctrl.mouse.press_l || rc_ctrl.rc.s[1] == 1)
	{
		target_speed[2] =  1500;
		motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
	}
	else
	{
		target_speed[2] = 0;
	 	motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
	}
}

//=============================================���ָ�==========================================================
static void magazine_task()
	
{

	if(r_flag)
	{
		target_speed[3] = -800;
	}
	else if(f_flag)//�رյ��ָ�
	{
		target_speed[3] =  800;
	}
	else //���ָǾ�ֹ
	{
		target_speed[3] = 0;
	}	
	motor_info[3].set_voltage = pid_calc(&motor_pid[3], target_speed[3], motor_info[3].rotor_speed);
}

static void magazine_init()
{
	pid_init(&motor_pid[3], 10 , 0.1, 0, 10000, 10000);
}

static void shoot_data_send()
{
	
    set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage,motor_info[2].set_voltage,motor_info[3].set_voltage);

    osDelay(1);		
}


//=============================================��������=============================================
static void heat_limit()
{
  if(infantry.heat_limit )
  {
	  if(infantry.shooter_heat > infantry.heat_limit )
	  {
		 	motor_info[2].set_voltage   = motor_info[2].set_voltage * 0;
		     
	  }
	  if(infantry.shooter_heat > infantry.heat_limit * 0.9)
	  {
			
		  	motor_info[2].set_voltage = motor_info[2].set_voltage * 0.5;
	  }
	  else
	  {
		//do nothing
	  }	  
  }
  else
  {
	  //do nothing
  }
}




