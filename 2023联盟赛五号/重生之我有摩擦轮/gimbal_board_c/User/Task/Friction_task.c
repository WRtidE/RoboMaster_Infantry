#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "Friction_task.h"
#include "Exchange_task.h"
#include "struct_typedef.h"

//PID初始化
static void Friction_init();

//模式定义
static void shoot_mode_1();
static void shoot_mode_2();
fp32 target_angle1;
//can发送
static void shoot_data_send();

void Friction_task(void const * argument)
{
  Friction_init();
  for(;;)
  {
	if((rc_ctrl.rc.s[1] == 2) || rc_ctrl.mouse.press_l) //发射
    {
		shoot_mode_1();
	}
	else //其余情况电机转速置0
	{
		shoot_mode_2();
	}
  }
  osDelay(1);
}

static void Friction_init()
{

}

static void shoot_mode_1()
{
	//此函数用来读取发射时的编码值，当初始化/pitch移动时记录编码电机编码值
	if( (shoot_flag == 0)||shoot_flag == 2)
    {
      target_angle1 = motor_info[4].rotor_angle;
    }

	pid_init(&motor_pid[0], 15, 0, 0, 16384, 16384); 
    pid_init(&motor_pid[1], 15, 0, 0, 16384, 16384); 
	pid_init(&motor_pid[2], 1, 0.01, 0, 16384, 16384);
	pid_init(&pitch_pid[4], 60, 1, 0, 16384, 16384);
    target_speed[0] = -7000;
    target_speed[1] =  7000;
    target_speed[2] = 800;

    motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
    motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
    motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
    motor_info[4].set_voltage = pid_calc(&pitch_pid[4], target_angle1, motor_info[4].rotor_angle);
    shoot_flag = 1;
	shoot_data_send();
	set_motor_voltage1(1, motor_info[4].set_voltage, 0,0,0);
}

static void shoot_mode_2()
{
	pid_init(&motor_pid[0], 30, 0, 0, 16384, 16384); 
	pid_init(&motor_pid[1], 30, 0, 0, 16384, 16384);
	pid_init(&motor_pid[2], 1, 0.01, 0, 16384, 16384);	
    target_speed[0] = 0;
    target_speed[1] = 0;
    target_speed[2] = 0;
    motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
    motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
    motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
    shoot_flag = 0;
	shoot_data_send();
}

static void shoot_data_send()
{
    set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage,motor_info[2].set_voltage,0);
    
    osDelay(1);		
}