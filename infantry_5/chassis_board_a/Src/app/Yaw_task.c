#include "Yaw_task.h"
#include "motor.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "cmsis_os.h"

fp32 ins_yaw;
fp32 ins_pitch;
fp32 ins_roll;
fp32 init_yaw;	
int yaw_model_flag = 1; 
fp32 err_yaw;		
fp32 angle_weight = 1;	
fp32 target_angle;


infantry_control chassis;
extern ins_data_t ins_data;
pid_struct_t yaw_pid[7];	

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


// static void Yaw_mode_3();



static void Yaw_can_send();


void gimbal_yaw_task(void const * argument)
{

	Yaw_init();
	

  for(;;)
  {
		Yaw_read_imu();

		if(rc_ctrl.rc.s[0] == 2 && ins_yaw)
		{
			Yaw_mode_1(); //假陀螺
		}
		else if(rc_ctrl.rc.s[0] == 3 || rc_ctrl.rc.s[0] == 1)
		{
			Yaw_mode_2(); //正常调整
		}
		// else if(rc_ctrl.rc.s[0] == 1)
		// {
		// 	Yaw_mode_3(); // 自瞄接口
		// }
    	osDelay(1);
  }

}
 


static void Yaw_init()	
{

	pid_init(&chassis.motor_speed_pid[4],5,0.01,0,30000,30000);
	pid_init(&motor_pid[4], 60, 1, 0, 30000, 30000); 
	pid_init(&yaw_pid[4], 0.1, 0.01, 0, 1000, 1000);  

}

static void Yaw_read_imu()
{
		ins_yaw = ins_data.angle[0];
		ins_pitch = ins_data.angle[1];
		ins_roll = ins_data.angle[2];
}



static void Yaw_mode_1()
{
	// if(rc_ctrl.rc.ch[0] > base-valve && rc_ctrl.rc.ch[0] < base+valve)
	// {
	// 	if(yaw_model_flag == 1)	
	// 	{
	// 		init_yaw = ins_roll;
	// 		yaw_model_flag = 0;
	// 	}

	// 	err_yaw = ins_roll - init_yaw;
	// 	if(err_yaw > angle_valve || err_yaw < -angle_valve)
	// 	{
	// 		motor_info[4].set_voltage = pid_calc(&yaw_pid[4], init_yaw, ins_roll);
	// 	}
	// 	else
	// 	{
	// 		motor_info[4].set_voltage = 0;
	// 	}
	// }
   	if(rc_ctrl.rc.ch[4]>=364&&rc_ctrl.rc.ch[4]<=1684)
   	{
   		target_speed[4] = -((rc_ctrl.rc.ch[0] - 1024) / 660 * 60);
   		motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
		yaw_model_flag = 1;
   	}
	else
	{
		if (rc_ctrl.rc.ch[0] >= 974 && rc_ctrl.rc.ch[0] <= 1074)
    	{
      		if (motor_info[4].rotor_speed > 10 || motor_info[4].rotor_speed < -10) 
      			{
        			target_speed[4] = 0;
        			motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
					yaw_model_flag = 1;
      			}
      		else
      			{
      			  motor_info[4].set_voltage = 0;
      			}
    	}
		else
   		{
   		  	if(rc_ctrl.rc.ch[0]>=364&&rc_ctrl.rc.ch[0]<=1684)
   		  	{
   		  	  target_speed[4] = -((rc_ctrl.rc.ch[0] - 1024) / 660 * 60);
   		  	  motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
			  yaw_model_flag = 1;
   		  	}
   		}
	}		
	
	set_motor_voltage1(1, motor_info[4].set_voltage, motor_info[5].set_voltage, motor_info[6].set_voltage, 0);
    osDelay(1);
}


static void Yaw_mode_2()
{
    if (rc_ctrl.rc.ch[0] >= 974 && rc_ctrl.rc.ch[0] <= 1074)
    {
      if (motor_info[4].rotor_speed > 10 || motor_info[4].rotor_speed < -10) 
      {
        target_speed[4] = 0;
        motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
		yaw_model_flag = 1;
      }
      else
      {
        motor_info[4].set_voltage = 0;
      }
    }else
    {
      if(rc_ctrl.rc.ch[0]>=364&&rc_ctrl.rc.ch[0]<=1684)
      {
        target_speed[4] = -((rc_ctrl.rc.ch[0] - 1024) / 660 * 60);
        motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
		yaw_model_flag = 1;
      }
    }
    set_motor_voltage1(1, motor_info[4].set_voltage, motor_info[5].set_voltage, motor_info[6].set_voltage, 0);
    osDelay(1);
}

// static void Yaw_mode_3()
// {
// 	target_angle = motor_info[4].rotor_angle - yaw_data * 8192/360;

// 	 motor_info[4].set_voltage = pid_calc(&yaw_pid[4], target_angle, motor_info[4].rotor_angle);

// 	set_motor_voltage1(1, motor_info[4].set_voltage, motor_info[5].set_voltage, motor_info[6].set_voltage, 0);

// }


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