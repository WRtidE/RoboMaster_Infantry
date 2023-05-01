#ifndef __PID_H
#define __PID_H
#include "main.h"
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
							
float pid_calc(pid_struct_t *pid, float ref, float fdb);						
#endif
							