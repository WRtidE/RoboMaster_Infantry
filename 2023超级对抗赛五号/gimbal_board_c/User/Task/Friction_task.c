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

fp32 count = 0;
fp32 heat_warning = 0;


//PID��ʼ��
static void Friction_init();

//ģʽѡ��
static void model_choice();

//ģʽ����
static void shoot_mode_1();
static void shoot_mode_2();

//����
static void magazine_task();
static void magazine_init();

//ǹ����������
static void heat_limit();

//can����
static void shoot_data_send();

//���⼤��ʹ��
static void laser_init();


void Friction_task(void const * argument)
{
  Friction_init();
  magazine_init();
  laser_init();
		
  for(;;)
  {	  
	model_choice();
	magazine_task();
	heat_limit();
	shoot_data_send();	  
  }
  osDelay(1);
}

static void Friction_init()
{

}

static void laser_init()
{
	HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_SET);
}
	
static void model_choice()
{
	if((rc_ctrl.rc.s[1] == 1) || rc_ctrl.mouse.press_l) //����
    {
		shoot_mode_1();
	}
	else //����������ת����0
	{
		shoot_mode_2();
	}
}

static void shoot_mode_1()
{

	pid_init(&motor_pid[0], 15, 0, 0, 16384, 16384); 
    pid_init(&motor_pid[1], 15, 0, 0, 16384, 16384); 
	pid_init(&motor_pid[2], 1 , 0.01, 0, 16384, 16384);

    target_speed[0] = -9000;
    target_speed[1] =  9000;
    target_speed[2] =  1600;
    motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
    motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
    motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
    
	shoot_flag = 1;
}

static void shoot_mode_2()
{	
	pid_init(&motor_pid[0], 30,   0, 0, 16384, 16384); 
	pid_init(&motor_pid[1], 30,   0, 0, 16384, 16384);
	pid_init(&motor_pid[2], 1, 0.01, 0, 16384, 16384);	

    target_speed[0] = 0;
    target_speed[1] = 0;
    target_speed[2] = 0;
    motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
    motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
    motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
	
	shoot_flag = 1;
}

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
}

static void magazine_init()
{
	pid_init(&motor_pid[3], 10 , 0.1, 0, 10000, 10000);
}

static void shoot_data_send()
{
    set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage,motor_info[2].set_voltage,motor_info[3].set_voltage);
    motor_info[3].set_voltage = pid_calc(&motor_pid[3], target_speed[3], motor_info[3].rotor_speed);
    osDelay(1);		
}


static void heat_limit()
{
  if(infantry.heat_limit && (!q_flag))
  {
	  if(infantry.shooter_heat > infantry.heat_limit )
	  {
		  motor_info[3].set_voltage *= 0;
		  
	  }
	  if(infantry.shooter_heat > infantry.heat_limit * 0.9)
	  {
		  motor_info[3].set_voltage *= 0.5;
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
