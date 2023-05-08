#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"
#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

typedef struct RC_ctrl_t
{
  struct rc
  {
    int16_t ch[5];
    char s[2];
  } rc;

  struct mouse
  {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
  } mouse;

  struct key
  {
    uint16_t v;
  } key;

} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;

extern volatile uint8_t rx_len_uart1;  //接收一帧数据的长度
extern volatile uint8_t recv_end_flag_uart1; //一帧数据接收完成标志
extern int16_t Yaw_minipc;
extern int16_t Pitch_minipc;
extern uint8_t rx_buffer[100];
extern void Get_minipc();
extern RC_ctrl_t rc_ctrl;
extern void remote_data_read(uint8_t rx_buffer[]);

extern void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3,int16_t v4);
extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern void set_motor_voltage1(uint8_t id_range, int16_t v1,int16_t v2,int16_t v3,int16_t v4);
extern void send_ins_data_to_a(int16_t ins_yaw,int16_t  ins_pitch,int16_t   ins_roll,int16_t  yaw_data);
#endif
