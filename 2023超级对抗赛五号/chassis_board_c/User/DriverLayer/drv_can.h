#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "struct_typedef.h"
#include "Chassis_task.h"
#include "INS_task.h"
void CAN1_Init(void);
void CAN2_Init( void );
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;
extern RC_ctrl_t rc_ctrl;
extern uint16_t w_flag;
extern uint16_t s_flag;
extern uint16_t a_flag;
extern uint16_t d_flag;
extern uint16_t q_flag;
extern uint16_t e_flag;
extern uint16_t shift_flag;
extern uint16_t ctrl_flag;
extern uint8_t press_left;
extern uint8_t press_right;
extern uint16_t r_flag;
extern uint16_t f_flag;
extern uint16_t g_flag;
extern uint16_t z_flag;
extern uint16_t x_flag;
extern uint16_t c_flag;
extern uint16_t v_flag;
extern uint16_t b_flag;
extern int16_t mouse_x;
extern CAN_TxHeaderTypeDef can1_TxHeader;
extern uint8_t can1_TxData[8];
extern CAN_TxHeaderTypeDef can2_TxHeader;
extern uint8_t can2_TxData[8];

extern CAN_RxHeaderTypeDef can1_RxHeader;
extern uint8_t can1_RxData[8];
extern CAN_RxHeaderTypeDef can2_RxHeader;
extern uint8_t can2_RxData[8];
extern void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
extern void set_motor_voltage1(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
extern ins_data_t ins_data1;

extern volatile int16_t motor_speed_target[5];
extern motor_info_t  motor_info_chassis[7];
extern pid_struct_t motor_pid_chassis[4];
#endif