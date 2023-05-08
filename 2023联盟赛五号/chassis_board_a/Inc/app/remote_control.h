#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#define __packed __attribute__((__packed__))
#include "can.h"

extern CAN_TxHeaderTypeDef can1_TxHeader;
extern uint8_t can1_TxData[8];
extern CAN_TxHeaderTypeDef can2_TxHeader;
extern uint8_t can2_TxData[8];

extern CAN_RxHeaderTypeDef can1_RxHeader;
extern uint8_t can1_RxData[8];
extern CAN_RxHeaderTypeDef can2_RxHeader;
extern uint8_t can2_RxData[8];

typedef struct __packed RC_ctrl_t
{
         struct __packed rc
        {
                int16_t ch[5];
                char s[2];
        } rc;
         struct __packed mouse
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
         struct __packed key
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;

extern void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
extern void set_motor_voltage1(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);

extern UART_HandleTypeDef huart6;
extern uint8_t rx_yaw[100];
extern volatile uint8_t rx_len_uart6;  //接收一帧数据的长度
extern volatile uint8_t recv_end_flag_uart6; //一帧数据接收完成标志

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
extern int16_t chassis_data;

#endif
