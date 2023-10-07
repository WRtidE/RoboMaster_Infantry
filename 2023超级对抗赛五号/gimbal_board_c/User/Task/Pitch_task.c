#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "remote_control.h"
#include "math.h"
#include "PID.h"
#include "INS_task.h"
//test
fp32 err;

//����һЩ����	
fp32 Err_pitch;
fp32 target_angle;
fp32 pitch_target_angle;

fp32 pitch_weight = 0.5;
uint32_t shoot_flag = 0;

//�ǶȻ�pid
pid_struct_t pitch_angle_pid;
//�ٶȻ�pid
pid_struct_t pitch_speed_pid;
//�����ٶȻ�pid
pid_struct_t aim_speed_pid;

extern ins_data_t ins_data;


//��ʼ��PID����
static void gimbal_init();	

//��ȡimu����
static void gimbal_read_imu();

//ѡ��ģʽ
static void gimbal_mode_1();
static void gimbal_mode_2();

//PID����ͷ���
static void gimbal_calc_and_send();

//����
static void auto_aim();

//Խ�紦��
static void err_calc();


// pitch
void Pitch_task(void const * argument)
{
	
	gimbal_init();
	osDelay(10000);
	for(;;)
	{
		Get_minipc(); //��ȡ��λ����������
		
		if(rc_ctrl.mouse.press_r||rc_ctrl.rc.s[0] == 1)//��������
		{
			auto_aim();
		}
		else
		{
			gimbal_mode_1();
		}
		
    	gimbal_calc_and_send();  
	}
	osDelay(1);
}

void gimbal_init()
{
	pid_init(&pitch_speed_pid, 85, 0, 10, 30000, 30000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 30000  
	pid_init(&aim_speed_pid,   20, 0, 0, 30000, 30000);
	pid_init(&pitch_angle_pid, 1, 0, 0, 1000, 1000);
	
	 pitch_target_angle = 2900;
	
	//�Ӿ��ӿ�
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //ʹ��IDLE�ж�
	HAL_UART_Receive_DMA(&huart1,rx_buffer,100); //��������,����minipc������
	
}


void auto_aim()
{
	if(Pitch_minipc_fp)
	{
		pitch_target_angle = motor_info[4].rotor_angle - Pitch_minipc_fp/360.0 * 8191; 
	}
	else
	{
		pitch_target_angle =  pitch_target_angle -((rc_ctrl.rc.ch[1]) / 660.0 * 2) + rc_ctrl.mouse.y / 16384.00 *1000; 
	}
	
	
	err_calc();
	
	target_speed[4] = pid_pitch_calc(&pitch_angle_pid,  pitch_target_angle,  motor_info[4].rotor_angle);
	
	osDelay(1);

	
}


void gimbal_mode_1()
{

	pitch_target_angle =  pitch_target_angle -((rc_ctrl.rc.ch[1]) / 660.0 * 10) + rc_ctrl.mouse.y / 16384.00 *1000; 
	
	err_calc();//�͸�����
	
	target_speed[4] = pid_pitch_calc(&pitch_angle_pid,  pitch_target_angle,  motor_info[4].rotor_angle);
	
	osDelay(1);
}

void gimbal_calc_and_send()
{
	if(rc_ctrl.mouse.press_r||rc_ctrl.rc.s[0] == 1)//��������
	{
		motor_info[4].set_voltage = pid_calc(&aim_speed_pid, target_speed[4], motor_info[4].rotor_speed);	
	}
	else
	{
		motor_info[4].set_voltage = pid_calc(&pitch_speed_pid, target_speed[4], motor_info[4].rotor_speed);
	}
	
	set_motor_voltage1( 1, motor_info[4].set_voltage, 0, 0, 0);
}

static void err_calc()
{
		if(pitch_target_angle > 8191)
	{
		pitch_target_angle -= 8191;
	}
	else if(pitch_target_angle < - 0)
	{
		pitch_target_angle += 8191;
	}
	
	if(pitch_target_angle<1500 )
	{
		pitch_target_angle = 1500;
	}
	else if (pitch_target_angle > 3600)
	{
		pitch_target_angle = 3600;
	}
}
