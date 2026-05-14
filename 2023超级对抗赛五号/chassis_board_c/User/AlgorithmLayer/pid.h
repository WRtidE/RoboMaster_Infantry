#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
typedef struct
{
    
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  
    fp32 max_iout; 

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  
    fp32 error[3]; 

} pid_struct_t;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	
	fp32 Dbuf[3]; 
	fp32 error[3]; 

    fp32 out;
} gimbal_PID_t;

extern void pid_init(pid_struct_t *pid, 
  fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 gimbal_PID_calc(pid_struct_t *pid, fp32 fdb, fp32 set);
extern void gimbal_PID_init(gimbal_PID_t *pid,fp32 PID[3], fp32 maxout, fp32 max_iout);



extern fp32 pid_calc(pid_struct_t *pid, fp32 ref, fp32 set);




#endif
