#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include  "drv_can.h"
#include  "drv_usart.h"
#include  "math.h"

//底盘电机状态


//陀螺仪矫正
fp32 imu_err = 0;

//电机pid
pid_struct_t motor_pid_chassis[4];
pid_struct_t cap;
 fp32 chassis_motor_pid [3]={10,0.5,10};   //用的原来的pid,30,0.5,10
 fp32 chassis_motor_pid2 [3]={40,1,10};   //用的原来的pid,30,0.5,10
 
 //超级电容pid
 fp32 cappid[3]={60,0,10};
 
 //超级电容变量
 fp32    cap_target; //使用超电
 fp32 no_cap_target; //不用超电
 fp32    target_cap;
 

volatile int16_t Vx=0,Vy=0,Wz=0;
volatile int16_t tempa = 2;

fp32 Temp_Vx;
fp32 Temp_Vy;
motor_info_t  motor_info_chassis[7];       //电机信息结构体
volatile int16_t motor_speed_target[5];


extern RC_ctrl_t rc_ctrl;
 
extern ins_data_t ins_data1; //云台角度
extern ins_data_t ins_data;  //底盘角度
// Save imu data
uint16_t Up_ins_yaw; 
uint16_t Down_ins_yaw = 268;		//必须赋这个初值，不知名Bug
uint16_t Down_ins_yaw_update = 180;

//云台坐标系
fp32 Err_yaw;	
fp32 Err_yaw_hudu;
fp32 Err_accident = 0;	//mechanical err 
fp32 Down_ins_pitch;
fp32 Down_ins_row;
fp64 sin_a;		
fp64 cos_a;

//功率限制算法的变量定义
fp32  speed_limit = 10000;
float Watch_Power_Max;
float Watch_Power;
float Watch_Buffer;
double Chassis_pidout;
double Chassis_pidout_target;
static double Scaling1=0,Scaling2=0,Scaling3=0,Scaling4=0;
float Klimit=1;
float Plimit=0;
float Chassis_pidout_max;


//用于键盘叠加
fp32 ramp[4] = {0,0,0,0};

//矫正陀螺仪
int16_t Drifting_yaw = 0;

//pid初始化
static void chassis_init();	

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

//mode3
static void Chassis_mode_3();

//模式选择
static void chassis_choice();

//陀螺仪矫正
static void imu_reset();

//超级电容
static void super_cap_task();

//键盘增量函数,让速度缓慢到达最大值
static void speed_ramp();
static void ramp_calc(fp32 ramp,uint16_t key,uint16_t speed_max);

//底盘功率限制
static void Chassis_Power_Limit(double Chassis_pidout_target_limit);
	
//速度限制
void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed);

//速度pid
void speed_pid_calc();




#define angle_valve 3
#define angle_weight 55
 
 
void Chassis_task(void const *pvParameters)
{
	chassis_init();
			
    for(;;)		
    {     
	 Chassis_loop_Init();
	 chassis_choice();                //控制模式选择
	 chassis_motol_speed_calculate(); //速度计算
	 speed_pid_calc();
	 Motor_Speed_limiting(motor_speed_target,speed_limit); //速度限制 
	 Chassis_Power_Limit(speed_limit * 4);      //功率限制
		//speed_pid_calc();
	 chassis_current_give();          //发送底盘电流
	 imu_reset();                     //重置陀螺仪
	
     osDelay(1);

    }  

}

static void chassis_init()
{
	
	for (uint8_t i = 0; i < 4; i++)
	{
        pid_init(&motor_pid_chassis[i], chassis_motor_pid, 10000, 10000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384			
	}   
}

void chassis_choice()
{
		
	
	if(rc_ctrl.rc.s[1] == 2 || ctrl_flag ) //小陀螺模式
	{
		Chassis_mode_2();
	}
	else if(rc_ctrl.rc.s[0] == 3||rc_ctrl.rc.s[0] == 1)
	{
		Chassis_mode_3(); //正常模式

	}
	else
	{
		Chassis_mode_1();//底盘跟随模式
	}
}


 //清除Vx，Vy，Vz的值
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
	
	Up_ins_yaw = ins_data1.angle[0] + imu_err;
	
	if(Up_ins_yaw > 360)
	{
		Up_ins_yaw -= 360;
	}
	else if(Up_ins_yaw < 0)
	{
		Up_ins_yaw += 360;
	}
	
	 
	Err_yaw = -(Up_ins_yaw -Down_ins_yaw);  //等效于 底盘 - 云台

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
	
	motor_speed_target[CHAS_LF] =   Vx+Vy+Wz;
    motor_speed_target[CHAS_RF] =   Vx-Vy+Wz;
    motor_speed_target[CHAS_RB] =  -Vy-Vx+Wz; 
    motor_speed_target[CHAS_LB] =   Vy-Vx+Wz;
	
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


	set_motor_voltage(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);
 
}

void speed_pid_calc()
{
	 for( uint8_t i=0 ; i<4; i++)
    {
        motor_info_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], motor_info_chassis[i].rotor_speed,motor_speed_target[i]);
    }	
}

static void Chassis_following()
{
	//阈值判断
	fp32 Wz_max = 5000;
	Get_Err();
	
	
	if(Err_yaw > angle_valve || Err_yaw < -angle_valve)
	{
		Wz -= Err_yaw * angle_weight;
	}
	
	
	if(Wz > Wz_max)
	{
		Wz = Wz_max;
	}
	else if(Wz < -Wz_max)
	{
		Wz = -Wz_max;
	}
}

static void Chassis_mode_1()
{
	//更新模式标志位
	infantry.chassis_free = 0;
	infantry.chassis_follow = 1;
	infantry.chassis_rovolve= 0;
	
	
	if( (rc_ctrl.rc.ch[2]>=-50&&rc_ctrl.rc.ch[2]<=50)&&((rc_ctrl.rc.ch[3]>=-50)&&(rc_ctrl.rc.ch[3]<=50))&&(rc_ctrl.rc.ch[4]<=50)&&(rc_ctrl.rc.ch[4]>=-50)
		&& ( !w_flag && !s_flag && !a_flag && !d_flag &&!q_flag && !e_flag) && (Err_yaw <= angle_valve) && (Err_yaw >= -angle_valve))
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
    else
    {
		speed_ramp();
        Vy=  rc_ctrl.rc.ch[3]/660.0*8000 +  ramp[0] -  ramp[1];
        Vx=  rc_ctrl.rc.ch[2]/660.0*8000 -  ramp[2] +  ramp[3];
       // Wz= -rc_ctrl.rc.ch[4]/660.0*8000 - q_flag/q_flag * 2000 + e_flag/e_flag * 2000;
    }
			
	Chassis_following();
}	

static void Chassis_mode_2() //小陀螺模式
{
	
	//更新模式标志位
	Get_Err();
	infantry.chassis_free = 0;
	infantry.chassis_follow = 0;
	infantry.chassis_rovolve= 1;
	
	Wz = 5000;

	Err_yaw_hudu = Err_yaw/57.3f; //添加负号的err相当于云台减底盘
			
	//calculate sin and cos
	cos_a = cos(Err_yaw_hudu);
	sin_a = sin(Err_yaw_hudu);
			

	// moving	control by remote

	 speed_ramp();
     Vy=  rc_ctrl.rc.ch[3]/660.0*8000 +  ramp[0] -  ramp[1];
     Vx=  rc_ctrl.rc.ch[2]/660.0*8000 -  ramp[2] +  ramp[3];

	//curl matrix * V
	Temp_Vx = Vx;
	Temp_Vy = Vy;
	Vx = Temp_Vx*cos_a - Temp_Vy*sin_a;
	Vy = Temp_Vx*sin_a + Temp_Vy*cos_a;		
		
}

static void Chassis_mode_3()
{
	//更新模式标志位
	infantry.chassis_free   = 1;
	infantry.chassis_follow = 0;
	infantry.chassis_rovolve= 0;
	if( (rc_ctrl.rc.ch[2]>=-50&&rc_ctrl.rc.ch[2]<=50)&&((rc_ctrl.rc.ch[3]>=-50)&&(rc_ctrl.rc.ch[3]<=50))&&(rc_ctrl.rc.ch[4]<=50)&&(rc_ctrl.rc.ch[4]>=-50)
		&& ( !w_flag && !s_flag && !a_flag && !d_flag && !q_flag && !e_flag) && (Err_yaw <= angle_valve) && (Err_yaw >= -angle_valve))
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
    else
    {
        speed_ramp();
        Vy=  rc_ctrl.rc.ch[3]/660.0*8000 +  ramp[0] -  ramp[1];
        Vx=  rc_ctrl.rc.ch[2]/660.0*8000 -  ramp[2] +  ramp[3];
       // Wz= -rc_ctrl.rc.ch[4]/660.0*8000 - q_flag/q_flag * 2000 + e_flag/e_flag * 2000;
    }
	
}

static void imu_reset()
{
	if(x_flag)
	{
		imu_err = ( 360 -(ins_data.angle[0] + 180)) - ins_data1.angle[0];  //底盘 - 云台
	}
}



static void speed_ramp()
{
	fp32 speed_max = 10000;
	uint8_t start = 10;
	uint16_t slow = 300;
	//前进
	if(w_flag)
	{
		 ramp[0] = ramp[0] + start;
	}
	else
	{
		 ramp[0] = ramp[0] - slow;
	}
	
	if(s_flag)
	{
		 ramp[1] = ramp[1] + start;
	}
	else
	{
		 ramp[1] = ramp[1] - slow;
	}
	
	if(a_flag)
	{
		 ramp[2] = ramp[2] + start;
	}
	else
	{
		 ramp[2] = ramp[2] - slow;
	}
	
	if(d_flag)
	{
		 ramp[3] = ramp[3] + start;
	}
	else
	{
		 ramp[3] = ramp[3] - slow;
	}
	
	for(int i =0 ;i < 4;i++)
	{
		if(ramp[i]>speed_max)
		{
			ramp[i] = speed_max;
		}
		else if(ramp[i] < 0)
		{
			ramp[i] = 0;
		}
	}
}


static void super_cap_task()
{

	target_cap = pid_calc(&cap,supercap_info.Vo ,14);
	if(shift_flag)
	{ 
		HAL_UART_Transmit_IT(&huart1,"PVONP",5);
		Motor_Speed_limiting(motor_speed_target,speed_limit +  target_cap); //速度限制 
		
	}
	else  
	{	
		HAL_UART_Transmit_IT(&huart1,"PVOFP",5);
		Motor_Speed_limiting(motor_speed_target,speed_limit); //速度限制 
	}
}



/*借鉴了防灾科技学院的算法，通过3个环来限制功率，有偏置，平滑曲线，缓冲能量约束的效果*/
//如果不行，试试把61536改成15384试试
static void Chassis_Power_Limit(double Chassis_pidout_target_limit)
{	
	//819.2/A，假设最大功率为120W，那么能够通过的最大电流为5A，取一个保守值：800.0 * 5 = 4000
	Watch_Power_Max = Klimit;	
	Watch_Power     = infantry.chassis_power;	
    Watch_Buffer    = infantry.chassis_power_buffer;//限制值，功率值，缓冲能量值，初始值是1，0，0
	//get_chassis_power_and_buffer(&Power, &Power_Buffer, &Power_Max);//通过裁判系统和编码器值获取（限制值，实时功率，实时缓冲能量）

		Chassis_pidout_max=61536;//32768，40，960			15384 * 4，取了4个3508电机最大电流的一个保守值

		if(Watch_Power>600)	Motor_Speed_limiting(motor_speed_target,4096);//限制最大速度 ;//5*4*24;先以最大电流启动，后平滑改变，不知道为啥一开始用的Power>960,可以观测下这个值，看看能不能压榨缓冲功率
	else{
		Chassis_pidout=(
						fabs(motor_speed_target[0]-motor_info_chassis[0].rotor_speed)+
						fabs(motor_speed_target[1]-motor_info_chassis[1].rotor_speed)+
						fabs(motor_speed_target[2]-motor_info_chassis[2].rotor_speed)+
						fabs(motor_speed_target[3]-motor_info_chassis[3].rotor_speed));//fabs是求绝对值，这里获取了4个轮子的差值求和
		
//	Chassis_pidout_target = fabs(motor_speed_target[0]) + fabs(motor_speed_target[1]) + fabs(motor_speed_target[2]) + fabs(motor_speed_target[3]);

		/*期望滞后占比环，增益个体加速度*/
		if(Chassis_pidout)
		{
		Scaling1=(motor_speed_target[0]-motor_info_chassis[0].rotor_speed)/Chassis_pidout;	
		Scaling2=(motor_speed_target[1]-motor_info_chassis[1].rotor_speed)/Chassis_pidout;
		Scaling3=(motor_speed_target[2]-motor_info_chassis[2].rotor_speed)/Chassis_pidout;	
		Scaling4=(motor_speed_target[3]-motor_info_chassis[3].rotor_speed)/Chassis_pidout;//求比例，4个scaling求和为1
		}
		else{Scaling1=0.25,Scaling2=0.25,Scaling3=0.25,Scaling4=0.25;}
		
		/*功率满输出占比环，车总增益加速度*/
//		if(Chassis_pidout_target) Klimit=Chassis_pidout/Chassis_pidout_target;	//375*4 = 1500
//		else{Klimit = 0;}
		Klimit = Chassis_pidout/Chassis_pidout_target_limit;
		
		if(Klimit > 1) Klimit = 1 ;
		else if(Klimit < -1) Klimit = -1;//限制绝对值不能超过1，也就是Chassis_pidout一定要小于某个速度值，不能超调

		/*缓冲能量占比环，总体约束*/
		if(Watch_Buffer<50&&Watch_Buffer>=40)	    Plimit=0.9;		//近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
		else if(Watch_Buffer<40&&Watch_Buffer>=35)	Plimit=0.75;
		else if(Watch_Buffer<35&&Watch_Buffer>=30)	Plimit=0.5;
		else if(Watch_Buffer<30&&Watch_Buffer>=20)	Plimit=0.25;
		else if(Watch_Buffer<20&&Watch_Buffer>=10)	Plimit=0.125;
		else if(Watch_Buffer<10&&Watch_Buffer>=0)	Plimit=0.05;
		else {Plimit=1;}
		
		motor_info_chassis[0].set_current = Scaling1*(Chassis_pidout_max*Klimit)*Plimit;//输出值
		motor_info_chassis[1].set_current = Scaling2*(Chassis_pidout_max*Klimit)*Plimit;
		motor_info_chassis[2].set_current = Scaling3*(Chassis_pidout_max*Klimit)*Plimit;
		motor_info_chassis[3].set_current = Scaling4*(Chassis_pidout_max*Klimit)*Plimit;/*同比缩放电流*/

	}

}