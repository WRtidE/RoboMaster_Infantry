#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H


typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32__t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32__t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

typedef struct 
{
	uint8_t Robot_level;
	uint8_t Robot_id;
	fp32 chassis_power;   //底盘功率
    fp32 chassis_power_buffer;  //底盘功率缓冲区
    fp32 total_current_limit;   //底盘输出电流限制
    fp32 total_current;          //底盘输出电流
	fp32 shooter_heat;			//枪口热量
}robot_data_t;

extern robot_data_t infantry;




#endif
