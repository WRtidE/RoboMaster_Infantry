#ifndef CAN_USER_H
#define CAN_USER_H

#include "main.h"
#include "remote_control.h"

void can_1_user_init(CAN_HandleTypeDef* hcan );
void can_2_user_init(CAN_HandleTypeDef* hcan );
void can_remote(uint8_t sbus_buf[],uint32_t id);
void can_rc_forward(RC_ctrl_t rc_ctrl);
extern void send_minipc_data_to_a(int16_t minipc_yaw);

typedef struct ROBOT_DATA
{
	uint8_t Robot_level;
	uint8_t Robot_id;
	fp32 shooter_heat;			//枪口热量
	fp32 heat_limit;			//枪口热量上限
	fp32 speed_limit;      //枪口射速上限
	

} robot_data_t;

extern robot_data_t infantry;
#endif