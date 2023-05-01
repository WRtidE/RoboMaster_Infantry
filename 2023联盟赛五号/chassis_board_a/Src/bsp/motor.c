#ifndef MOTOR_BOT_H
#define MOTOR_BOT_H
#include "struct_typedef.h"
#include "PID.h"
#include "remote_control.h"



/*使用指南*/
/*（后续完善）*/



//接收电机的返回值
typedef struct
{
    uint16_t angle;           // 角度
    int16_t speed_rpm;      //  电机转速
    int16_t torque_current;  // 给定电流
    uint8_t temperate;     
    int16_t last_angle;
} motor_recieve_t;


//单个电机数据结构体
typedef struct
{
  motor_recieve_t motor_recieve; // 电机的基本数据
  fp32 accel;     // 加速度
  fp32 speed;	  // 速度
  fp32 target_speed; // 设定速度
  int16_t set_voltage;  // 电压赋值 
  fp32 target_angle;

  
  fp32 max_angle;    //角度最大值
  fp32 min_angle;    //角度最小值
  fp32 mid_angle;    //角度中值

} infantry_motor_t;

//infantry total data
//步兵整体控制结构体
typedef struct 
{
    infantry_motor_t motor[8];           //电机结构体
    pid_struct_t motor_speed_pid[8];     //电机pid结构体
} infantry_control;

infantry_control   gimbal;

#endif