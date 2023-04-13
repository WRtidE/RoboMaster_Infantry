#ifndef YAW_TASK_H
#define YAW_TASK_H


#include "pid.h"
#include "struct_typedef.h"
#include "remote_control.h"

	
extern RC_ctrl_t rc_ctrl;		


void gimbal_yaw_task(void const * argument);

#endif