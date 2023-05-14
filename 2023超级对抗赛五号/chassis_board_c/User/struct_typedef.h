#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H


typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

typedef struct
{
    uint16_t can_id;		//ID号
    int16_t  set_current;		//发送信息
    uint16_t rotor_angle;		//现在的角度
    int16_t  rotor_speed;		//现在的转速
    int16_t  torque_current;		//实际转矩电流
    uint8_t  temp;		//电机温度
}motor_info_t;

typedef struct ROBOT_DATA
{
	//===========裁判系统读取数据==================
	uint8_t Robot_level;
	uint8_t Robot_id;
	fp32 chassis_power;   //底盘功率
    fp32 chassis_power_buffer;  //底盘功率缓冲区
    fp32 total_current_limit;   //底盘输出电流限制
    fp32 total_current;          //底盘输出电流
	fp32 shooter_heat;			//枪口热量
	//================用户定义数据================
	
	fp32 shoot_frequent;         //射频（和拨盘转速有关）
	

} robot_data_t;

extern robot_data_t infantry;

#endif



