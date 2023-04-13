#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H


typedef signed char int8_t;
typedef signed short int int16_t;
//typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
//typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;


typedef struct INS_DATA
{
    fp32 accel_offset[3];
    fp32 gyro_offset[3];
    fp32 accel[3];
    fp32 temp;
    fp32 gyro[3];
    fp32 angle[3];
    fp32 INS_quat[4];  
} ins_data_t;

ins_data_t ins_data;

fp32 yaw_data;

#endif



