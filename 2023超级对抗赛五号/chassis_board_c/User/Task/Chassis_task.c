#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include  "drv_can.h"
extern float newpower;
extern float powerdata[4];
pid_struct_t motor_pid_chassis[4];
pid_struct_t cap;
motor_info_t  motor_info_chassis[7];       //电机信息结构体
 fp32 chassis_motor_pid [3]={40,0.5,10};   //用的原来的pid,30,0.5,10
 fp32 chassis_motor_pid2 [3]={40,1,10};   //用的原来的pid,30,0.5,10
 fp32 cappid[3]={60,0,10};
volatile int16_t Vx=0,Vy=0,Wz=0;
 volatile int16_t tempa = 2;
volatile int16_t notcaptarget;
int16_t Temp_Vx;
int16_t Temp_Vy;
volatile int16_t motor_speed_target[5];
volatile int16_t captarget;

extern RC_ctrl_t rc_ctrl;
 
extern ins_data_t ins_data1;
extern ins_data_t ins_data;
// Save imu data
uint16_t Up_ins_yaw; 
 uint16_t Down_ins_yaw = 268;		//必须赋这个初值，不知名Bug
uint16_t Down_ins_yaw_update = 180;

fp32 Err_yaw;	
fp32 Err_yaw_hudu;
fp32 Err_accident = 0;	//mechanical err 
fp32 Down_ins_pitch;
fp32 Down_ins_row;
fp32 sin_a;		
fp32 cos_a;
int8_t chassis_choice_flag = 0;
int8_t chassis_mode = 1;
int flag[1] = {0};//是否使用超级电容，否填0，是填1

//矫正陀螺仪
int16_t Drifting_yaw = 0;

//获取imu——Yaw角度差值参数
static void Get_Err(); 

//参数重置
static void Chassis_loop_Init(); 

//底盘跟随云台
static void Chassis_following();

//mode1
static void Chassis_mode_1();

//mode2
static void Chassis_mode_2();

//模式选择
static void Chassis_choice();
	
#define angle_valve 5
#define angle_weight 55
 
   void Chassis_task(void const *pvParameters)
{
 	for (uint8_t i = 0; i < 4; i++)
	{
        pid_init(&motor_pid_chassis[i], chassis_motor_pid, 4000, 4000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384			
	}   
			//pid_init(&motor_pid_chassis[1], chassis_motor_pid2, 10000, 10000);
			//pid_init(&motor_pid_chassis[3], chassis_motor_pid2, 3000, 3000);
			pid_init(&cap, cappid, 2000, 2000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
			
    for(;;)				//3000 3000 2500
    {     
 						captarget = pid_calc(&cap,14,(int16_t)powerdata[1]);
			notcaptarget = ((24.f + (newpower/0.15f + 100)) / 24.f + 0.2f)*300.f;
            RC_to_Vector();                          //遥控器信息转换为底盘速度Vy,Vx,Wz  Remote controller information converted to chassis speed Vy, Vx, Wz
            chassis_motol_speed_calculate(); 			//电机速度计算，即麦轮运动解算      Calculation of Mecanum Wheel Motion
			if(flag[0]==0)
			{
            Motor_Speed_limiting(motor_speed_target,20000);//限制最大速度                     limit maximum speed
			}
						if(flag[0]==1)
			{
						 
				
            //Motor_Speed_limiting(motor_speed_target,3300+captarget);//限制最大速度                     limit maximum speed
				Motor_Speed_limiting(motor_speed_target,notcaptarget);
			}
								if(flag[0]==2)
			{
						
            Motor_Speed_limiting(motor_speed_target,4500+captarget);//限制最大速度                     limit maximum speed
			}
			
			else
			{
				Motor_Speed_limiting(motor_speed_target,20000);//限制最大速度                     limit maximum speed
				
			}
				
			
            chassis_current_give();                 //发送电流                         send current
            osDelay(1);

    }



}

void RC_to_Vector()
{
		Chassis_loop_Init();
		Get_Err();
		// flag for right pattern 
	if(rc_ctrl.rc.s[1]==3 || rc_ctrl.rc.s[1]==1)//左上角拨杆从上到下132
    
    {
			Chassis_choice();
			if(chassis_mode == 1)
			{
				Chassis_mode_1();
			}
			else if(chassis_mode == 2)
			{
				Chassis_mode_2();
			}
			
    }
		
		//pattern 2 ( Constantly curl + moving )小陀螺
		else if(rc_ctrl.rc.s[1]==2)			
		{
			Chassis_mode_2();
		}
		

	
 }

static void Chassis_loop_Init()
{
	Vx = 0;
	Vy = 0;
	Wz = 0;
}

static void Get_Err()
{
			Down_ins_yaw = 360 -(ins_data.angle[0] + 180);
			Down_ins_pitch = ins_data.angle[1];
			Down_ins_row = ins_data.angle[2];
	
			//校正陀螺仪漂移
//			Down_ins_yaw_update = Down_ins_yaw - Drifting_yaw;
			
			//calculate err of yaw
			Up_ins_yaw = ins_data1.angle[0];
			Err_yaw = -(Up_ins_yaw -Down_ins_yaw);
			//Err_yaw = Err_yaw/57.3f;
		
			//越界处理,保证转动方向不变
			if(Err_yaw < -180)	//	越界时：180 -> -180
			{
				Err_yaw += 360;
			}
					
			else if(Err_yaw > 180)	//	越界时：-180 -> 180
			{
				Err_yaw -= 360;
			}
}

void chassis_motol_speed_calculate()
{
	
	motor_speed_target[CHAS_LF] =  Vx+Vy+Wz;
    motor_speed_target[CHAS_RF] =  Vx-Vy+Wz;
    motor_speed_target[CHAS_RB] =  -Vy-Vx+Wz; 
    motor_speed_target[CHAS_LB] =  Vy-Vx+Wz;
}

  void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed)  
{
    uint8_t i=0;
    int16_t max = 0;
    int16_t temp =0;
    int16_t max_speed = limit_speed;
    fp32 rate=0;
    for(i = 0; i<4; i++)
    {
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//求绝对值
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
        {
            motor_speed[i] *= rate;
        }

    }

}

void chassis_current_give() 
{
	
    uint8_t i=0;
        
    for(i=0 ; i<4; i++)
    {
        motor_info_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], motor_info_chassis[i].rotor_speed,motor_speed_target[i]);
    }
    	set_motor_voltage(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);
 

}

static void Chassis_following()
{
	//先检测云台已经停止转动


			//阈值判断
			if(Err_yaw > angle_valve || Err_yaw < -angle_valve)
			{
				Wz -= Err_yaw * angle_weight;
			}


}

static void Chassis_mode_1()
{

      
			
			//braking to stop quickly
			if( (rc_ctrl.rc.ch[2]>=-50&&rc_ctrl.rc.ch[2]<=50)&&((rc_ctrl.rc.ch[3]>=-50)&&(rc_ctrl.rc.ch[3]<=50))&&(rc_ctrl.rc.ch[4]<=50)&&(rc_ctrl.rc.ch[4]>=-50)
				&& ( !w_flag && !s_flag && !a_flag && !d_flag) && (Err_yaw <= angle_valve) && (Err_yaw >= -angle_valve))
			{

				for(int i=0;i<4;i++)//减速  slow_down
				{
                 
					if(motor_info_chassis[i].rotor_speed>360||motor_info_chassis[i].rotor_speed<-360)
					{
						motor_info_chassis[i].set_current = pid_calc(&motor_pid_chassis[i],  motor_info_chassis[i].rotor_speed, 0);
					}
					else
					{
						motor_info_chassis[i].set_current=0;
					}
				}

			}
    
		// moving	control by remote
    else if( !w_flag && !s_flag && !a_flag && !d_flag)
    {
			

        Vy= rc_ctrl.rc.ch[3]/660.0*8000;
        Vx= rc_ctrl.rc.ch[2]/660.0*8000;
        Wz= -rc_ctrl.rc.ch[4]/660.0*8000;

    }
		
		// moving control by keyboard
		else
		{
		
				if(w_flag)
				{
					Vy = 9158;
				}
				else if(s_flag)
				{
					Vy = -9158;
				}
				else
				{
					Vy = 0;
				}
				
				if(a_flag)
				{
					Vx = -9158;
				}
				else if(d_flag)
				{
					Vx = 9158;
				}	
				else 
				{
					Vx = 0;
				}
			

		}

				Chassis_following();
}	

static void Chassis_mode_2()
{
			if(flag[0]==2)//狂暴模式
			{
						
            Wz = 5000+captarget/3;
			}
			//const a number for curling
			else{
			Wz = 20000+captarget/3;}
			
			
//			// Read imu data
//			Down_ins_yaw = ins_data.angle[0] + 180;
//			Down_ins_pitch = ins_data.angle[1];
//			Down_ins_row = ins_data.angle[2];
//			
//			//校正陀螺仪漂移
//			Down_ins_yaw_update = Down_ins_yaw - Drifting_yaw;
//	
//			//calculate err of yaw
//			Err_yaw = Down_ins_yaw_update - Down_ins_yaw;
//			//Err_yaw = test_up_ins_yaw - Down_ins_yaw;
			Err_yaw_hudu = Err_yaw/57.3f;
			
			
			//calculate sin and cos
			cos_a = cos(Err_yaw_hudu);
			sin_a = sin(Err_yaw_hudu);
			
			
			
		
			

		// moving	control by remote
    if( !w_flag && !s_flag && !a_flag && !d_flag)
    {
    
        Vy= rc_ctrl.rc.ch[3]/660.0*8000;
        Vx= rc_ctrl.rc.ch[2]/660.0*8000;
			
			
				//curl matrix * V
				Temp_Vx = Vx;
				Temp_Vy = Vy;
				Vx = Temp_Vx*cos_a - Temp_Vy*sin_a;
				Vy = Temp_Vx*sin_a + Temp_Vy*cos_a;
				Vx = Vx/tempa;
				Vy = Vy/tempa;
			

    }
				// moving control by keyboard
		else 
		{
	
			
				if(w_flag)
				{
					Vy = 9158;
				}
				else if(s_flag)
				{
					Vy = -9158;
				}
				else
				{
					Vy = 0;
				}
				
				if(a_flag)
				{
					Vx = -9158;
				}
				else if(d_flag)
				{
					Vx = 9158;
				}	
				
				else 
				{
					Vx = 0;
				}
				
				
				//curl matrix * V
				Temp_Vx = Vx;
				Temp_Vy = Vy;
				Vx = Temp_Vx*cos_a - Temp_Vy*sin_a;
				Vy = Temp_Vx*sin_a + Temp_Vy*cos_a;
				Vx = Vx/tempa;
				Vy = Vy/tempa;
				
		}
		
}

static void Chassis_choice()
{
	if(r_flag)
	{
		chassis_choice_flag = 1;
	}
	if(shift_flag)
	{
		flag[0] = 2;
	}
	else
	{
		flag[0] = 1;
	}
	
	if( (!r_flag) && (chassis_choice_flag == 1) )	
	{
		chassis_choice_flag = 0;
		if(chassis_mode == 1)
		{
			chassis_mode = 2;
		}
		else if(chassis_mode == 2)
		{
			chassis_mode = 1;
		}
	}
	if(r_flag)
	{
		chassis_choice_flag = 1;
	}
	if(q_flag)
	{
		Wz = 1000;
	}
	if(e_flag)
	{
		Wz = -1000;
	
	}
}


