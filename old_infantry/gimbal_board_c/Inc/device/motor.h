#ifndef MOTOR_H
#define MOTOR_H
#include "can.h"
#include "struct_typedef.h"

extern CAN_TxHeaderTypeDef can1_TxHeader;
extern uint8_t can1_TxData[8];
extern CAN_TxHeaderTypeDef can2_TxHeader;
extern uint8_t can2_TxData[8];

extern CAN_RxHeaderTypeDef can1_RxHeader;
extern uint8_t can1_RxData[8];
extern CAN_RxHeaderTypeDef can2_RxHeader;
extern uint8_t can2_RxData[8];

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;





#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

extern void CAN_rc_forward(int16_t ch[], char s[]);
extern void CAN_cmd_chassis(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void send_ins_data_to_a(int32_t ins_yaw, int32_t ins_pitch, int32_t ins_roll);
#endif