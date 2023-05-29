#ifndef YAW_TASK_H
#define YAW_TASK_H

#include "struct_typedef.h"
#include "pid.h"
#include  "drv_can.h"
#include "rc_potocal.h"
#include "main.h"


extern pid_struct_t motor_pid_chassis[4];
extern int16_t Drifting_yaw;
extern uint16_t Down_ins_yaw;
extern fp32 yaw_data;

void Chassis_task(void const *pvParameters);
void RC_to_Vector(void);
void chassis_motol_speed_calculate(void);
void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed);
void chassis_current_give(void);	
extern fp32 ins_yaw;

void gimbal_yaw_task(void const * argument);

#endif