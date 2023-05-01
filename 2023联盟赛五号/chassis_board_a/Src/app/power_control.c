#include "power_control.h"
#include "judge.h"
#include "math.h"

#define POWER_LIMIT         80.0f   //功率限制
#define POWER_BUFF_LIMIT    60.0f	//缓冲区限制  
#define WARNING_POWER       40.0f   //警戒功率
#define WARNING_POWER_BUFF  50.0f   //再多一点就会爆炸

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 没有功率和电流限制的情况
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f    //缓冲区电流限制
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f    //电流限制

/*
电机的电流值乘以24V（底盘供电电压）近似等于该电机的功率
限制最后的输出电流:计算PID，获得四个电机的电流目标值，其和若大于当前可用功率限制，
则同比例增大/缩小速度。再次计算，直至总功率小于限制值*/

void chassis_power_control()
{
    

	fp32 chassis_power 		  = infantry.chassis_power;  //底盘功率
    fp32 chassis_power_buffer = infantry.chassis_power_buffer; //底盘功率缓冲区
    fp32 total_current_limit  = 0.0f;   //底盘输出电流限制
    fp32 total_current 		  = 0.0f;         //底盘输出电流
    
	if(infantry.Robot_id == 0)
	{
		 total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT; //如果未连接裁判系统，就不进功率限制
	}
	else
	{
			//功率超过80w 和缓冲能量小于60j,因为缓冲能量小于60意味着功率超过80w
		if(chassis_power_buffer < WARNING_POWER_BUFF)   //缓冲能量小于设定值，说明超功率
		{
			fp32 power_scale;
			if(chassis_power_buffer > 5.0f)
			{
				//缩小WARNING_POWER_BUFF
				power_scale = chassis_power_buffer / WARNING_POWER_BUFF;       //如果缓冲区剩余量>5.0，比例 = 当前缓冲区占用值/总缓冲区值
			}
			else
			{
				//only left 10% of WARNING_POWER_BUFF
				power_scale = 5.0f / WARNING_POWER_BUFF;      //如果缓冲区能量剩余 < 5.0f ,情况十分紧急，强制限速至原速度的0.1
			}
			//scale down
			//缩小
			total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;  //当前电流限制 = 总电流限制 * 减小比例 
		}
		else
		{
			//power > WARNING_POWER
			//功率大于WARNING_POWER
			if(chassis_power > WARNING_POWER) //功率 > 超功率预警
			{
				fp32 power_scale;

				//power < 80w
				//功率小于80w，未超功率
				if(chassis_power < POWER_LIMIT) //未超功率，但超过了警戒功率
				{
					//scale down
					//缩小
					power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
					
				}
				//power > 80w
				//功率大于80w，超功率
				else
				{
					power_scale = 0.0f; 
				}
				
				total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
			}
			//power < WARNING_POWER
			//功率小于WARNING_POWER
			else
			{
				total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
			}
		}
	}


    
    total_current = 0.0f;
    //calculate the original motor current set
    //计算原本电机电流设定
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(motor_pid[i].output); //返回浮点数的绝对值
    }
    

    if(total_current > total_current_limit) //如果总电流设定超过上限，就进行限速
    {
        fp32 current_scale = total_current_limit / total_current;
        motor_pid[0].output = current_scale;
		motor_pid[1].output = current_scale;
		motor_pid[2].output = current_scale;
		motor_pid[3].output = current_scale;
    }
}
