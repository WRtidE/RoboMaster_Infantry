#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "remote_control.h"
#include "math.h"
#include "PID.h"
#include "INS_task.h"


//此任务用来对云台进行模式选择，控制，校准等
//封装一些函数用来进行控制调用
//Pitch采用上C板CAN_2，电机ID为5
//存在问题：需要将云台Pitch锁在最下方再启动，Pitch受高频噪点影响，收敛速度太慢

//定义一些变量	



//imu数据
fp32 Err_pitch;
int16_t Up_pitch;
int16_t Down_pitch;
extern ins_data_t ins_data;
 fp32 target_angle;
 
 uint32_t shoot_flag = 0;
 pid_struct_t pitch_pid[7];

//初始化PID参数
static void gimbal_init();	

//校验连接成功
static bool gimbal_judge();	

//读取imu参数
static void gimbal_read_imu();

//模式选择
static void gimbal_choice();

//选择模式
static void gimbal_mode_1();
static void gimbal_mode_2();

//PID计算和发送
static void gimbal_can_send();

//限位（相对陀螺仪数据）
static void gimbal_imu_limit();

//反转限位
static void gimbal_imu_limit_2();

//鼠标控制Pitch(叠加)
static void gimbal_mouse();

//叠加自瞄
static void gimbal_minipc_control();
// pitch
void Pitch_task(void const * argument)
{
  pid_init(&motor_pid[4], 80, 3, 0, 30000, 30000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 30000  
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能IDLE中断
	HAL_UART_Receive_DMA(&huart1,rx_buffer,100); //开启接收
  for(;;)
  { 
    Get_minipc();
    //send_ins_data_to_a(0,0,0,Yaw_minipc);
    if(rc_ctrl.rc.s[0] == 1)
    {
        target_angle = motor_info[4].rotor_angle - Pitch_minipc* 8192/360;

	    motor_info[4].set_voltage = pid_calc(&pitch_pid[4], target_angle, motor_info[4].rotor_angle);
		  
		set_motor_voltage1(1, motor_info[4].set_voltage, 0, 0, 0);//改完这个 摩擦轮好了
    }
	  if (!rc_ctrl.mouse.y)
	  {
		  if (rc_ctrl.rc.ch[1] >= -50 && rc_ctrl.rc.ch[1] <= 50) 
		  {
				if (motor_info[4].rotor_speed < - 10 || motor_info[4].rotor_speed > 10) //锟斤拷锟斤拷(?)
				{
					target_speed[4] = 0;
					motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
				}
				else
				{
					target_speed[4] = 0;
				}
		  }
			else
		  {
				if(rc_ctrl.rc.ch[1]>=-660&&rc_ctrl.rc.ch[1]<=-200 && (motor_info[4].rotor_angle < 1500 ||  motor_info[4].rotor_angle > 6000)) //遥控器往下拨//遥控器往下拨
				{
					target_speed[4] = -((rc_ctrl.rc.ch[1]) / 660.0 * 16);
					motor_info[4].set_voltage =  pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
					shoot_flag = 2;
				}
				if(rc_ctrl.rc.ch[1]<=660 && (motor_info[4].rotor_angle > 7000 || motor_info[4].rotor_angle < 3000 )) //遥控器往上拨
				{
					target_speed[4] = -((rc_ctrl.rc.ch[1]) / 660.0*16);
					motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
					shoot_flag = 2;
				}
		  }
	  }
	  else
	  {
		  if (rc_ctrl.mouse.y >= -5 && rc_ctrl.mouse.y <= 5) 
		  {
				if (motor_info[4].rotor_speed < - 10 || motor_info[4].rotor_speed > 10) //锟斤拷锟斤拷(?)
				{
					target_speed[4] = 0;
					motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
				}
				else
				{
					target_speed[4] = 0;
				}
		  }
		  else
		  {
				if(rc_ctrl.mouse.y <-5 && rc_ctrl.mouse.y >=-16384 && (motor_info[4].rotor_angle < 1500 ||  motor_info[4].rotor_angle > 6000)) //遥控器往下拨//遥控器往下拨
				{
					target_speed[4] =  rc_ctrl.mouse.y / 16384.00 * 3000;
					motor_info[4].set_voltage =  pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
					shoot_flag = 2;
				}
				if(rc_ctrl.mouse.y>5 && rc_ctrl.mouse.y <= 16384 && (motor_info[4].rotor_angle > 7000 || motor_info[4].rotor_angle < 3000 )) //遥控器往上拨
				{
					target_speed[4] =  rc_ctrl.mouse.y / 16384.00 *3000;
					motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
					shoot_flag = 2;
				}
		  }
	  }
      set_motor_voltage1( 1, motor_info[4].set_voltage, 0, 0, 0);

    osDelay(1);
  }
}