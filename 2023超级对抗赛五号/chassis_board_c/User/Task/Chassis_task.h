#ifndef CHASSIS_TASK_H
#define  CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include  "drv_can.h"
#include "rc_potocal.h"
#include "main.h"



//id   3   4
//     2   1
typedef enum {
	CHAS_RB,  
	CHAS_LB,
    CHAS_LF,      
    CHAS_RF,
} chassis_motor_cnt_t;


//extern pid_struct_t motor_pid_chassis[4];
extern int16_t Drifting_yaw;
extern uint16_t Down_ins_yaw;
//motor_info_t  motor_info_chassis[8];

void Chassis_task(void const *pvParameters);
void RC_to_Vector(void);
void chassis_motol_speed_calculate(void);
void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed);
void chassis_current_give(void);
#endif
