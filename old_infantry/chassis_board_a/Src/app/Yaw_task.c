#include "Yaw_task.h"
#include "motor.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "cmsis_os.h"

fp32 ins_yaw;
fp32 ins_pitch;
fp32 ins_row;
fp32 init_yaw;	
int yaw_model_flag = 1; 
fp32 err_yaw;		
fp32 angle_weight = 1;	


infantry_control chassis;
extern ins_data_t ins_data;

#define valve 100	
#define base 1024		
#define base_max 1684		
#define base_min 364
#define angle_valve 10		

static void Yaw_init();	


static void Yaw_read_imu();


// static void Yaw_choice();


static void Yaw_mode_1();


static void Yaw_mode_2();

static void Yaw_can_send();

void gimbal_yaw_task(void const * argument)
{

	Yaw_init();
	

  for(;;)
  {
	Yaw_read_imu();

	if(rc_ctrl.rc.s[0] == 2 && ins_yaw)
	{
		Yaw_mode_1();
	}
		
	else if(rc_ctrl.rc.s[0] == 1 || rc_ctrl.rc.s[0] == 3)
	{
		Yaw_mode_2();
	}
    osDelay(1);
  }

}
 


static void Yaw_init()	
{

	pid_init(&chassis.motor_speed_pid[4],5,0.01,0,30000,30000);
}

static void Yaw_read_imu()
{
	ins_yaw = ins_data.angle[0];
	ins_pitch = ins_data.angle[1];
	ins_row = ins_data.angle[2];
}


// 传递值是0~360

// 170 - 180  = -10
// 270 - 180  =  90 


void Yaw_mode_1()
{
	if(rc_ctrl.rc.ch[0] > base-valve && rc_ctrl.rc.ch[0] < base+valve) //遥控器在中值
	{
		if(yaw_model_flag == 1)	  // 记录当前角度
		{
			init_yaw = ins_yaw;
			yaw_model_flag = 0;
		}
		
		err_yaw = ins_yaw - init_yaw;	//误差角
		
		
		// if(err_yaw < -180)	
		// {
		// 	err_yaw += 360;    
		// }
		
		// else if(err_yaw > 180)
		// {
		// 	err_yaw -= 360;      
		// }
	
		
		if(err_yaw > angle_valve || err_yaw < -angle_valve) 
		{
			// chassis.motor[4].target_speed = err_yaw * angle_weight;
			chassis.motor[4].target_speed = - (err_yaw * angle_weight);
		}
		
		else
		{
			chassis.motor[4].target_speed = 0;
		}
	}
	
	if(rc_ctrl.rc.ch[0] >= base+valve && rc_ctrl.rc.ch[0] <= base_max)    
	{
		chassis.motor[4].target_speed = 60;
		yaw_model_flag = 1;
	}
	else if(rc_ctrl.rc.ch[0] >= base_min && rc_ctrl.rc.ch[0]<base - valve)  
	{
		chassis.motor[4].target_speed = 60;
		yaw_model_flag = 1;
	}
					
	
	chassis.motor[4].set_voltage = pid_calc(&chassis.motor_speed_pid[4], chassis.motor[4].target_speed , chassis.motor[4].motor_recieve.speed_rpm);
	Yaw_can_send();
}

void Yaw_mode_2()
{
	
	if(rc_ctrl.rc.ch[0] >= 1224 && rc_ctrl.rc.ch[0] <= 1684)  
	{
    	chassis.motor[4].target_speed = 50;
	}
	
	else if(rc_ctrl.rc.ch[0] >= 364 && rc_ctrl.rc.ch[0]<824)  
	{
    	chassis.motor[4].target_speed = -50;
	}
	
	else
	{
    	chassis.motor[4].target_speed = 0;
	}
	chassis.motor[4].set_voltage = pid_calc(&chassis.motor_speed_pid[4], chassis.motor[4].target_speed, chassis.motor[4].motor_recieve.speed_rpm);
	Yaw_can_send();
}




static void Yaw_can_send()
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
  tx_header.StdId = 0x2ff;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;		

  tx_data[0] = (chassis.motor[4].set_voltage>>8)&0xff;		
  tx_data[1] = (chassis.motor[4].set_voltage)&0xff;
  tx_data[2] = 0x00;
  tx_data[3] = 0x00;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}