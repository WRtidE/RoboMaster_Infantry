#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "remote_control.h"
#include "math.h"
#include "PID.h"
#include "INS_task.h"


//��������������̨����ģʽѡ�񣬿��ƣ�У׼��
//��װһЩ�����������п��Ƶ���
//Pitch������C��CAN_2�����IDΪ5
//�������⣺��Ҫ����̨Pitch�������·���������Pitch�ܸ�Ƶ���Ӱ�죬�����ٶ�̫��

//����һЩ����	



//imu����
fp32 Err_pitch;
int16_t Up_pitch;
int16_t Down_pitch;
extern ins_data_t ins_data;
 fp32 target_angle;
 
 uint32_t shoot_flag = 0;
 pid_struct_t pitch_pid[7];

//��ʼ��PID����
static void gimbal_init();	

//У�����ӳɹ�
static bool gimbal_judge();	

//��ȡimu����
static void gimbal_read_imu();

//ģʽѡ��
static void gimbal_choice();

//ѡ��ģʽ
static void gimbal_mode_1();
static void gimbal_mode_2();

//PID����ͷ���
static void gimbal_can_send();

//��λ��������������ݣ�
static void gimbal_imu_limit();

//��ת��λ
static void gimbal_imu_limit_2();

//������Pitch(����)
static void gimbal_mouse();

//��������
static void gimbal_minipc_control();
// pitch
void Pitch_task(void const * argument)
{
  pid_init(&motor_pid[4], 80, 3, 0, 30000, 30000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 30000  
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //ʹ��IDLE�ж�
	HAL_UART_Receive_DMA(&huart1,rx_buffer,100); //��������
  for(;;)
  { 
    Get_minipc();
    //send_ins_data_to_a(0,0,0,Yaw_minipc);
    if(rc_ctrl.rc.s[0] == 1)
    {
        target_angle = motor_info[4].rotor_angle - Pitch_minipc* 8192/360;

	    motor_info[4].set_voltage = pid_calc(&pitch_pid[4], target_angle, motor_info[4].rotor_angle);
		  
		set_motor_voltage1(1, motor_info[4].set_voltage, 0, 0, 0);//������� Ħ���ֺ���
    }
	  if (!rc_ctrl.mouse.y)
	  {
		  if (rc_ctrl.rc.ch[1] >= -50 && rc_ctrl.rc.ch[1] <= 50) 
		  {
				if (motor_info[4].rotor_speed < - 10 || motor_info[4].rotor_speed > 10) //����(?)
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
				if(rc_ctrl.rc.ch[1]>=-660&&rc_ctrl.rc.ch[1]<=-200 && (motor_info[4].rotor_angle < 1500 ||  motor_info[4].rotor_angle > 6000)) //ң�������²�//ң�������²�
				{
					target_speed[4] = -((rc_ctrl.rc.ch[1]) / 660.0 * 16);
					motor_info[4].set_voltage =  pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
					shoot_flag = 2;
				}
				if(rc_ctrl.rc.ch[1]<=660 && (motor_info[4].rotor_angle > 7000 || motor_info[4].rotor_angle < 3000 )) //ң�������ϲ�
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
				if (motor_info[4].rotor_speed < - 10 || motor_info[4].rotor_speed > 10) //����(?)
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
				if(rc_ctrl.mouse.y <-5 && rc_ctrl.mouse.y >=-16384 && (motor_info[4].rotor_angle < 1500 ||  motor_info[4].rotor_angle > 6000)) //ң�������²�//ң�������²�
				{
					target_speed[4] =  rc_ctrl.mouse.y / 16384.00 * 3000;
					motor_info[4].set_voltage =  pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
					shoot_flag = 2;
				}
				if(rc_ctrl.mouse.y>5 && rc_ctrl.mouse.y <= 16384 && (motor_info[4].rotor_angle > 7000 || motor_info[4].rotor_angle < 3000 )) //ң�������ϲ�
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