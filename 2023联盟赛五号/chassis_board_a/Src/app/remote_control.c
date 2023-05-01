#include "remote_control.h"
#include "struct_typedef.h"





#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"


#include "remote_control.h"
#include "math.h"
#include "PID.h"
#define YAW_SIZE 100
uint8_t temp = 0;
RC_ctrl_t rc_ctrl;
uint16_t rc_tmp[5], rc_flag = 0;

CAN_TxHeaderTypeDef can1_TxHeader;
uint8_t can1_TxData[8];
CAN_TxHeaderTypeDef can2_TxHeader;
uint8_t can2_TxData[8];

CAN_RxHeaderTypeDef can1_RxHeader;
uint8_t can1_RxData[8];
CAN_RxHeaderTypeDef can2_RxHeader;
uint8_t can2_RxData[8];

// flag for keyboard
uint16_t w_flag;
uint16_t s_flag;
uint16_t a_flag;
uint16_t d_flag;
uint16_t q_flag;
uint16_t e_flag;
uint16_t shift_flag;
uint16_t ctrl_flag;
uint8_t press_left;
uint8_t press_right;
uint16_t r_flag;
uint16_t f_flag;
uint16_t g_flag;
uint16_t z_flag;
uint16_t x_flag;
uint16_t c_flag;
uint16_t v_flag;
uint16_t b_flag;
//鼠标向右为正
int16_t mouse_x; 



void get_yaw_data();
void chassis_data_read();
extern UART_HandleTypeDef huart6;
int16_t chassis_data=0;
uint8_t rx_yaw[100]={0};

//把rc_ctrl.key.v中的按键数据读取出来
void get_key();
// 此处代码非自动生成
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // Can 接收中断回调
    if (hcan == &hcan1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1_RxHeader, can1_RxData); //将数据传输到RxData中

        if (can1_RxHeader.StdId == 0x300 && rc_flag == 0)
        {
            rc_tmp[0] = (can1_RxData[0] << 8) | can1_RxData[1];
            rc_tmp[1] = (can1_RxData[2] << 8) | can1_RxData[3];
            rc_tmp[2] = (can1_RxData[4] << 8) | can1_RxData[5];
            rc_tmp[3] = (can1_RxData[6] << 8) | can1_RxData[7];
            rc_flag = 1;
        }
        else if (can1_RxHeader.StdId == 0x301 && rc_flag == 1)
        {
            rc_ctrl.rc.ch[0] = rc_tmp[0];
            rc_ctrl.rc.ch[1] = rc_tmp[1];
            rc_ctrl.rc.ch[2] = rc_tmp[2];
            rc_ctrl.rc.ch[3] = rc_tmp[3];
            rc_ctrl.rc.ch[4] = (can1_RxData[0] << 8) | can1_RxData[1];
            rc_ctrl.rc.s[0] = can1_RxData[2];
            rc_ctrl.rc.s[1] = can1_RxData[3];
			
		    w_flag=(can1_RxData[5]&0x01);
			s_flag=(can1_RxData[5]&0x02);
			a_flag=(can1_RxData[5]&0x04);
			d_flag=(can1_RxData[5]&0x08);
			q_flag=(can1_RxData[5]&0x40);
			e_flag=(can1_RxData[5]&0x80);
			shift_flag=(can1_RxData[5]&0x10);
			ctrl_flag=(can1_RxData[5]&0x20);
		
			r_flag=(can1_RxData[4]&0x01);
			f_flag=(can1_RxData[4]&0x02);
			g_flag=(can1_RxData[4]&0x04);
			z_flag=(can1_RxData[4]&0x08);
			x_flag=(can1_RxData[4]&0x10);
			c_flag=(can1_RxData[4]&0x20);
			v_flag=(can1_RxData[4]&0x40);
			b_flag=(can1_RxData[4]&0x80);
						
			mouse_x = (can1_RxData[6] << 8) | can1_RxData[7];
		
            rc_flag = 0;
        }
        else if(can1_RxHeader.StdId == 0x302 )
        {
            //接收c板陀螺仪数据
            ins_data.angle[0] = (can1_RxData[0] << 8) | can1_RxData[1];
            ins_data.angle[1] = (can1_RxData[2] << 8) | can1_RxData[3];
            ins_data.angle[2] = (can1_RxData[4] << 8) | can1_RxData[5];
            yaw_data = (can1_RxData[6] << 8) | can1_RxData[7];
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

    // HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
}
/*
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
*/
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = (id_range == 0) ? (0x200) : (0x1ff); //如果id_range==0则等于0x200,id_range==1则等于0x1ff（ID号）
    tx_header.IDE = CAN_ID_STD;                            //标准帧
    tx_header.RTR = CAN_RTR_DATA;                          //数据帧
    tx_header.DLC = 8;                                     //发送数据长度（字节）

    tx_data[0] = (v1 >> 8) & 0xff; //先发高八位
    tx_data[1] = (v1)&0xff;
    tx_data[2] = (v2 >> 8) & 0xff;
    tx_data[3] = (v2)&0xff;
    tx_data[4] = (v3 >> 8) & 0xff;
    tx_data[5] = (v3)&0xff;
    tx_data[6] = (v4 >> 8) & 0xff;
    tx_data[7] = (v4)&0xff;
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
    osDelay(1);
}


void set_motor_voltage1(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
    //6020电机输出
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = (id_range == 0) ? (0x1ff) : (0x2ff); //如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
    tx_header.IDE = CAN_ID_STD;                            //标准帧
    tx_header.RTR = CAN_RTR_DATA;                          //数据帧
    tx_header.DLC = 8;                                     //发送数据长度（字节）

    tx_data[0] = (v1 >> 8) & 0xff; //先发高八位
    tx_data[1] = (v1)&0xff;
    tx_data[2] = (v2 >> 8) & 0xff;
    tx_data[3] = (v2)&0xff;
    tx_data[4] = (v3 >> 8) & 0xff;
    tx_data[5] = (v3)&0xff;
    tx_data[6] = (v4 >> 8) & 0xff;
    tx_data[7] = (v4)&0xff;
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
    osDelay(1);
}

void get_key()
{
		r_flag = rc_ctrl.key.v & (0x00 | 0x01 << 8);
		f_flag = rc_ctrl.key.v & (0x00 | 0x02 << 8);
		g_flag = rc_ctrl.key.v & (0x00 | 0x04 << 8);
		z_flag = rc_ctrl.key.v & (0x00 | 0x08 << 8);
		x_flag = rc_ctrl.key.v & (0x00 | 0x10 << 8);
		c_flag = rc_ctrl.key.v & (0x00 | 0x20 << 8);
		v_flag = rc_ctrl.key.v & (0x00 | 0x40 << 8);
		b_flag = rc_ctrl.key.v & (0x00 | 0x80 << 8);
}

void get_yaw_data()
{
		if(recv_end_flag_uart6 == 1)  //接收完成标志
		{			
			if(rx_yaw[0] == 's')//改了下标志位
			{
				chassis_data_read(rx_yaw);
			}
			recv_end_flag_uart6 = 0;//清除接收结束标志位
			for(uint8_t i=0;i<rx_len_uart6;i++)
				{
					rx_yaw[i]=0;//清接收缓存
				}
				//memset(rx_buffer,0,rx_len);
			rx_len_uart6 = 0;//清除计数
			HAL_UART_Receive_DMA(&huart6,rx_yaw,YAW_SIZE);//重新打开DMA接收
			
		}

}
void chassis_data_read(uint8_t rx_yaw[])
{
	chassis_data = (int)(rx_yaw[1] << 8 | rx_yaw[2]);
}


