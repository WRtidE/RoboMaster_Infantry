#ifndef CHASSIS_TASK_H
#define  CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include "main.h"

//id   3   4
//     2   1
typedef enum {
	CHAS_RB,  
	CHAS_LB,
    CHAS_LF,      
    CHAS_RF,
} chassis_motor_cnt_t;


#endif
