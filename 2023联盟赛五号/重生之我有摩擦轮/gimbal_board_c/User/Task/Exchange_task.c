#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "INS_task.h"
#include "Exchange_task.h"



//Some flag for keyboard

uint16_t w_flag;
uint16_t s_flag;
uint16_t a_flag;
uint16_t d_flag;
uint16_t q_flag;
uint16_t e_flag;
uint16_t shift_flag;
uint16_t ctrl_flag;
uint8_t press_left;
uint8_t press_right;
uint16_t r_flag;
uint16_t f_flag;
uint16_t g_flag;
uint16_t z_flag;
uint16_t x_flag;
uint16_t c_flag;
uint16_t v_flag;
uint16_t b_flag;

uint8_t temp_test;
//向右和向下为正
int16_t mouse_x;
int16_t mouse_y;
//extern int16_t mouse_y;

extern RC_ctrl_t rc_ctrl;
extern ins_data_t ins_data;
uint8_t ins_buf[8];
uint16_t ins_buf_temp;		//define a temp to receive and change float to int16
 

#define BUFFER_SIZE 100
extern UART_HandleTypeDef huart1;
volatile uint8_t rx_len_uart1 = 0;  //接收一帧数据的长度
volatile uint8_t recv_end_flag_uart1 = 0; //一帧数据接收完成标志
uint8_t rx_buffer[100]={0};  //接收数据缓存数组
int16_t Yaw_minipc;
int16_t Pitch_minipc;


void Get_keyboard();
void Get_minipc();
void remote_data_read(uint8_t rx_buffer[]);

void Exchange_task(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	
	ins_buf[0] = 8;	//imu receive tag
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能IDLE中断
	HAL_UART_Receive_DMA(&huart1,rx_buffer,100); //开启接收
	
  for(;;)
  {
//		ins_buf_temp = ins_data.angle[0] + 180;		// Add 180 to be a positive number
//		ins_buf[1] = ins_buf_temp;
//		ins_buf[2] = ins_buf_temp >> 8;		// 2 bytes put together forming a uint_16
//		can_remote(ins_buf,0x55);
		
		Get_keyboard();		//解算keyboard flag	
		Get_minipc();
		
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
} 

void Get_keyboard()	
{
		w_flag=rc_ctrl.key.v & (0x01 | 0x00 << 8);
		s_flag=rc_ctrl.key.v & (0x02 | 0x00 << 8);
		a_flag=rc_ctrl.key.v & (0x04 | 0x00 << 8);
		d_flag=rc_ctrl.key.v & (0x08 | 0x00 << 8);
		q_flag=rc_ctrl.key.v & (0x40 | 0x00 << 8);
		e_flag=rc_ctrl.key.v & (0x80 | 0x00 << 8);
		shift_flag=rc_ctrl.key.v & (0x10 | 0x00 << 8);
		ctrl_flag=rc_ctrl.key.v & (0x20 | 0x00 << 8);
		press_left = rc_ctrl.mouse.press_l;
		press_right = rc_ctrl.mouse.press_r;
	  mouse_x = rc_ctrl.mouse.x;
		mouse_y = rc_ctrl.mouse.y;
	
		r_flag = rc_ctrl.key.v & (0x00 | 0x01 << 8);
		f_flag = rc_ctrl.key.v & (0x00 | 0x02 << 8);
		g_flag = rc_ctrl.key.v & (0x00 | 0x04 << 8);
		z_flag = rc_ctrl.key.v & (0x00 | 0x08 << 8);
		x_flag = rc_ctrl.key.v & (0x00 | 0x10 << 8);
		c_flag = rc_ctrl.key.v & (0x00 | 0x20 << 8);
		v_flag = rc_ctrl.key.v & (0x00 | 0x40 << 8);
		b_flag = rc_ctrl.key.v & (0x00 | 0x80 << 8);
		
}

void Get_minipc()
{

		if(recv_end_flag_uart1 == 1)  //接收完成标志
		{			
			if(rx_buffer[0] == 0x01)
			{
				remote_data_read(rx_buffer);
			}
			recv_end_flag_uart1 = 0;//清除接收结束标志位
			for(uint8_t i=0;i<rx_len_uart1;i++)
				{
					rx_buffer[i]=0;//清接收缓存
				}
				//memset(rx_buffer,0,rx_len);
			rx_len_uart1 = 0;//清除计数
			HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);//重新打开DMA接收
			
		}
}

void remote_data_read(uint8_t rx_buffer[])
{
	Yaw_minipc = (int)(rx_buffer[1] << 8 | rx_buffer[2]);
	Pitch_minipc = (int)(rx_buffer[3] << 8 | rx_buffer[4]);
	Yaw_minipc = (int)(Yaw_minipc * 100)/32767;
	Pitch_minipc = (int)(Pitch_minipc * 100)/32767;
}