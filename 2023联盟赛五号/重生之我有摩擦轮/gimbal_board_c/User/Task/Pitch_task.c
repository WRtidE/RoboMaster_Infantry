#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "remote_control.h"
#include "math.h"
#include "PID.h"
#include "INS_task.h"

//定义一些变量	

fp32 Err_pitch;
fp32 target_angle;
int8_t pitch_weight = 3;
uint32_t shoot_flag = 0;
pid_struct_t pitch_pid[7];
extern ins_data_t ins_data;

//初始化PID参数
static void gimbal_init();	

//读取imu参数
static void gimbal_read_imu();

//选择模式
static void gimbal_mode_1();
static void gimbal_mode_2();

//PID计算和发送
static void gimbal_calc_and_send();

//自瞄
static void auto_aim();


// pitch
void Pitch_task(void const * argument)
{
	gimbal_init();
	for(;;)
	{
<<<<<<< HEAD
		if(rc_ctrl.mouse.press_r||rc_ctrl.rc.s[0] == 1)//开启自瞄
=======
		if(rc_ctrl.mouse.press_r||rc_ctrl.rc.s[1] == 1)//开启自瞄
>>>>>>> master
		{
			auto_aim();
		}
		else
		{
			gimbal_mode_1();
		}
	}
	osDelay(1);
}

void gimbal_init()
{
	pid_init(&motor_pid[4], 80, 3, 0, 30000, 30000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 30000  
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能IDLE中断
	HAL_UART_Receive_DMA(&huart1,rx_buffer,100); //开启接收,接受minipc的数据
	

}

//void auto_aim()
//{
//	Get_minipc();
//	
//	target_angle = motor_info[4].rotor_angle - Pitch_minipc* 8192/360;
//	motor_info[4].set_voltage = pid_calc(&pitch_pid[4], target_angle, motor_info[4].rotor_angle);	
//	set_motor_voltage1(1, motor_info[4].set_voltage, 0, 0, 0);
//	
//	shoot_flag = 2;
//	
//}

void auto_aim()
{
	Get_minipc();
	if(Pitch_minipc != 0)
	{
		target_speed[4] = Pitch_minipc *  pitch_weight;
	
<<<<<<< HEAD
		gimbal_calc_and_send();
=======
		//gimbal_calc_and_send();
>>>>>>> master
		shoot_flag = 2;
	}
	else
	{
		
	}

	
}


void gimbal_mode_1()
{
	
  if (!rc_ctrl.mouse.y)
  {
	  if (rc_ctrl.rc.ch[1] >= -50 && rc_ctrl.rc.ch[1] <= 50) 
	  {
		target_speed[4] = 0;
	  }
	  else
	  {
		if(rc_ctrl.rc.ch[1] >= -660&&rc_ctrl.rc.ch[1]<=-200 && (motor_info[4].rotor_angle < 1500 ||  motor_info[4].rotor_angle > 6500)) //遥控器往下拨//遥控器往下拨
		{
			target_speed[4]  = -((rc_ctrl.rc.ch[1]) / 660.0 * 16);
			shoot_flag = 2;
		}
		if(rc_ctrl.rc.ch[1] <= 660 && (motor_info[4].rotor_angle > 7000 || motor_info[4].rotor_angle < 3000 )) //遥控器往上拨
		{
			target_speed[4] = -((rc_ctrl.rc.ch[1]) / 660.0*16);
			shoot_flag = 2;
		}
	  }
  }
  else
  {
	  if (rc_ctrl.mouse.y >= -5 && rc_ctrl.mouse.y <= 5) 
	  {
		target_speed[4] = 0;
	  }
	  else
	  {
		if(rc_ctrl.mouse.y <-5 && rc_ctrl.mouse.y >=-16384 && (motor_info[4].rotor_angle < 1500 ||  motor_info[4].rotor_angle > 6500)) //遥控器往下拨//遥控器往下拨
		{
			target_speed[4] =  rc_ctrl.mouse.y / 16384.00 * 3000;
			shoot_flag = 2;
		}
		if(rc_ctrl.mouse.y>5 && rc_ctrl.mouse.y <= 16384 && (motor_info[4].rotor_angle > 7000 || motor_info[4].rotor_angle < 3000 )) //遥控器往上拨
		{
			target_speed[4] =  rc_ctrl.mouse.y / 16384.00 *3000;
			shoot_flag = 2;
		}
	  }
  }
<<<<<<< HEAD
  gimbal_calc_and_send();
=======
  // gimbal_calc_and_send();
>>>>>>> master
  osDelay(1);
}

void gimbal_calc_and_send()
{
	motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
	set_motor_voltage1( 1, motor_info[4].set_voltage, 0, 0, 0);
}