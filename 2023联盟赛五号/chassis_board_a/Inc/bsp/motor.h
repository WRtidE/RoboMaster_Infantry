#ifndef MOTOR_H
#define MOTOR_H
#include "struct_typedef.h"
#include "PID.h"
#include "remote_control.h"


typedef struct
{
    uint16_t angle;           
    int16_t speed_rpm;      
    int16_t torque_current;  
    uint8_t temperate;     
    int16_t last_angle;
} motor_recieve_t;


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


typedef struct 
{
    infantry_motor_t motor[8];           
    pid_struct_t motor_speed_pid[8];     
} infantry_control;

extern infantry_control   chassis;



#endif
