#include "Can_user.h"
#include "remote_control.h"
#include "struct_typedef.h"

CAN_TxHeaderTypeDef can1_TxHeader;
uint8_t can1_TxData[8];
CAN_TxHeaderTypeDef can2_TxHeader;
uint8_t can2_TxData[8];

CAN_RxHeaderTypeDef can1_RxHeader;
uint8_t can1_RxData[8];
CAN_RxHeaderTypeDef can2_RxHeader;
uint8_t can2_RxData[8];
uint8_t test[8];

uint16_t rc_tmp[5];

robot_data_t infantry;


void can_1_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;                       // filter 0
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;//标识符寄存器 
  can_filter.FilterIdLow  = 0;//标识符寄存器 
  can_filter.FilterMaskIdHigh = 0;//屏蔽寄存器
  can_filter.FilterMaskIdLow  = 0;       //屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
	HAL_CAN_Start(&hcan1);//启动can，封装在can_user_init()里了
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);		//使能can的FIFO0中断，也封装在can_user_init()里了
}

void can_2_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 14;                       // filter 14
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;//标识符寄存器 
  can_filter.FilterIdLow  = 0;//标识符寄存器 
  can_filter.FilterMaskIdHigh = 0;//屏蔽寄存器
  can_filter.FilterMaskIdLow  = 0;       //屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
	HAL_CAN_Start(&hcan2);//启动can，封装在can_user_init()里了
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);		//使能can的FIFO0中断，也封装在can_user_init()里了
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // Can 接收中断回调
    if (hcan == &hcan1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1_RxHeader, can1_RxData); //将数据传输到RxData中
        if (can1_RxHeader.StdId == 0x305)
        {
             infantry.shooter_heat = (can1_RxData[0] << 8) | can1_RxData[1]; //枪口热量
             infantry.heat_limit   = (can1_RxData[2] << 8) | can1_RxData[3]; //枪口热量上限
             infantry.Robot_level =  can1_RxData[4]; //机器人等级
         }
    }
    else if (hcan == &hcan2)
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (hcan->Instance == CAN2)
        {
            HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data); // receive can data
        }
        if ((rx_header.StdId >= FEEDBACK_ID_BASE)                    // 201-207
            && (rx_header.StdId < FEEDBACK_ID_BASE + MOTOR_MAX_NUM)) // 判断标识符，标识符为0x200+ID
        {
            uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE; // get motor index by can_id
            motor_info[index].rotor_angle = ((rx_data[0] << 8) | rx_data[1]);
            motor_info[index].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
            motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
            motor_info[index].temp = rx_data[6];
        }if ((rx_header.StdId >= FEEDBACK_ID_BASE_6020)                    // 205-211,注意把ID调成大于3,不然就会和读取3508的函数产生冲突
            && (rx_header.StdId < FEEDBACK_ID_BASE_6020 + MOTOR_MAX_NUM)) // 判断标识符，标识符为0x204+ID
        {
            uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE_6020; // get motor index by can_id
            motor_info[index].rotor_angle = ((rx_data[0] << 8) | rx_data[1]);
            motor_info[index].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
            motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
            motor_info[index].temp = rx_data[6];
        }
    }

}


void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;

    can2_TxHeader.StdId = 0x300;
    can2_TxHeader.IDE = CAN_ID_STD;
    can2_TxHeader.RTR = CAN_RTR_DATA;
    can2_TxHeader.DLC = 0x08;
    can2_TxData[0] = motor1 >> 8;
    can2_TxData[1] = motor1;
    can2_TxData[2] = motor2 >> 8;
    can2_TxData[3] = motor2;
    can2_TxData[4] = motor3 >> 8;
    can2_TxData[5] = motor3;
    can2_TxData[6] = motor4 >> 8;
    can2_TxData[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &can2_TxHeader, can2_TxData, &send_mail_box);
}

void can_rc_forward(RC_ctrl_t rc_ctrl)
{
    //c板发送遥控器数据 分俩帧
    uint32_t send_mail_box;

    can1_TxHeader.StdId = 0x300;
    can1_TxHeader.IDE = CAN_ID_STD;
    can1_TxHeader.RTR = CAN_RTR_DATA;
    can1_TxHeader.DLC = 0x08;
    can1_TxData[0] = rc_ctrl.rc.ch[0] >> 8;
    can1_TxData[1] = rc_ctrl.rc.ch[0];
    can1_TxData[2] = rc_ctrl.rc.ch[1] >> 8;
    can1_TxData[3] = rc_ctrl.rc.ch[1];
    can1_TxData[4] = rc_ctrl.rc.ch[2] >> 8;
    can1_TxData[5] = rc_ctrl.rc.ch[2];
    can1_TxData[6] = rc_ctrl.rc.ch[3] >> 8;
    can1_TxData[7] = rc_ctrl.rc.ch[3];
    HAL_CAN_AddTxMessage(&hcan1, &can1_TxHeader, can1_TxData, &send_mail_box);

    can1_TxHeader.StdId = 0x301;
    can1_TxHeader.IDE = CAN_ID_STD;
    can1_TxHeader.RTR = CAN_RTR_DATA;
    can1_TxHeader.DLC = 0x08;
    can1_TxData[0] = rc_ctrl.rc.ch[4] >> 8;
    can1_TxData[1] = rc_ctrl.rc.ch[4];
    can1_TxData[2] = rc_ctrl.rc.s[0];
    can1_TxData[3] = rc_ctrl.rc.s[1];
    can1_TxData[4] = rc_ctrl.key.v >> 8;
    can1_TxData[5] = rc_ctrl.key.v;
    can1_TxData[6] = rc_ctrl.mouse.x >> 8;
    can1_TxData[7] = rc_ctrl.mouse.x;
    HAL_CAN_AddTxMessage(&hcan1, &can1_TxHeader, can1_TxData, &send_mail_box);
	
//	can1_TxHeader.StdId = 0x36;
//    can1_TxHeader.IDE = CAN_ID_STD;
//    can1_TxHeader.RTR = CAN_RTR_DATA;
//    can1_TxHeader.DLC = 0x08;
//    can1_TxData[0] = rc_ctrl.mouse.x >> 8;
//    can1_TxData[1] = rc_ctrl.mouse.x;
//    can1_TxData[2] = rc_ctrl.mouse.y >> 8;
//    can1_TxData[3] = rc_ctrl.mouse.y;
//    can1_TxData[4] = rc_ctrl.mouse.z >> 8;	
//    can1_TxData[5] = rc_ctrl.mouse.z;
//    can1_TxData[6] = rc_ctrl.mouse.press_l;
//    can1_TxData[7] = rc_ctrl.mouse.press_r;
//    HAL_CAN_AddTxMessage(&hcan1, &can1_TxHeader, can1_TxData, &send_mail_box);
		
}



//can发送陀螺仪数据和minipc数据
void send_ins_data_to_a(int16_t ins_yaw, int16_t ins_pitch, int16_t ins_roll,int16_t yaw_data)
{
    uint32_t send_mail_box;
	
	can1_TxHeader.StdId = 0x302;
    can1_TxHeader.IDE = CAN_ID_STD;
    can1_TxHeader.RTR = CAN_RTR_DATA;
    can1_TxHeader.DLC = 0x08;
    test[0] = (ins_yaw >> 8) & 0xFF;
    test[1] = ins_yaw & 0xFF;
    test[2] = (ins_pitch >> 8) & 0xFF;
    test[3] = ins_pitch & 0xFF;
    test[4] = (ins_roll >> 8) & 0xFF;
    test[5] = ins_roll & 0xFF;
    test[6] = (yaw_data >> 8) & 0xFF;
    test[7] = yaw_data & 0xFF;
    HAL_CAN_AddTxMessage(&hcan1, &can1_TxHeader, test, &send_mail_box);
}
