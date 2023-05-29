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
    uint16_t can_id;		//ID��
    int16_t  set_current;		//������Ϣ
    uint16_t rotor_angle;		//���ڵĽǶ�
    int16_t  rotor_speed;		//���ڵ�ת��
    int16_t  torque_current;		//ʵ��ת�ص���
    uint8_t  temp;		//����¶�
}motor_info_t;

typedef struct ROBOT_DATA
{
	uint8_t Robot_level;
	uint8_t Robot_id;
	fp32 chassis_power;   //���̹���
    fp32 chassis_power_buffer;  //���̹��ʻ�����
    fp32 total_current_limit;   //���������������
    fp32 total_current;          //�����������
	fp32 shooter_heat;			//ǹ������
	fp32 shooter_heat_limit;
	fp32 shoot_speed_limit; //ǹ����������
	
	
	
	//������״̬
	uint8_t chassis_follow;  //���̸���ģʽ
	uint8_t chassis_free;    //��������ģʽ
	uint8_t chassis_rovolve; //С����
} robot_data_t;

extern robot_data_t infantry;
#endif



