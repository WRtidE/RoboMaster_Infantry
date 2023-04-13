#include "motor.h"
#include "main.h"
#include "struct_typedef.h"
#include "cmsis_os.h"


CAN_TxHeaderTypeDef can1_TxHeader;
uint8_t can1_TxData[8];
CAN_TxHeaderTypeDef can2_TxHeader;
uint8_t can2_TxData[8];

CAN_RxHeaderTypeDef can1_RxHeader;
uint8_t can1_RxData[8];
CAN_RxHeaderTypeDef can2_RxHeader;
uint8_t can2_RxData[8];

uint16_t rc_tmp[5];

// 此处代码非自动生成
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    // Can 发送中断回调
    if (hcan == &hcan2)
    {
        HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
        // usart_printf("can2 transported\n");
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // Can 接收中断回调
    if (hcan == &hcan1)
    {
        // HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1_RxHeader, can1_RxData); //将数据传输到RxData中
        // //usart_printf("%d\n",can1_RxHeader.StdId);
        // if (can1_RxHeader.StdId == 0x300)
        // {
        //     rc_tmp[0] = (can1_RxData[0] << 8) | can1_RxData[1];
        //     rc_tmp[1] = (can1_RxData[2] << 8) | can1_RxData[3];
        //     rc_tmp[2] = (can1_RxData[4] << 8) | can1_RxData[5];
        //     rc_tmp[3] = (can1_RxData[6] << 8) | can1_RxData[7];
        // }
        // else if (can1_RxHeader.StdId == 0x301)
        // {
        //     usart_printf("ch0:%d,ch1:%d,ch2:%d,ch3:%d,ch4:%d\n", rc_tmp[0], rc_tmp[1], rc_tmp[2], rc_tmp[3], ((can1_RxData[0] << 8) | can1_RxData[1]), can1_RxData[2], can1_RxData[3]);
        // }
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

    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
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

void CAN_rc_forward(int16_t ch[], char s[],int32_t ins_yaw, int32_t ins_pitch, int32_t ins_roll,int32_t yaw_data)
{
    //c板发送遥控器数据 分俩帧
    uint32_t send_mail_box;

    can1_TxHeader.StdId = 0x300;
    can1_TxHeader.IDE = CAN_ID_STD;
    can1_TxHeader.RTR = CAN_RTR_DATA;
    can1_TxHeader.DLC = 0x08;
    can1_TxData[0] = ch[0] >> 8;
    can1_TxData[1] = ch[0];
    can1_TxData[2] = ch[1] >> 8;
    can1_TxData[3] = ch[1];
    can1_TxData[4] = ch[2] >> 8;
    can1_TxData[5] = ch[2];
    can1_TxData[6] = ch[3] >> 8;
    can1_TxData[7] = ch[3];

    HAL_CAN_AddTxMessage(&hcan1, &can1_TxHeader, can1_TxData, &send_mail_box);

    can1_TxHeader.StdId = 0x301;
    can1_TxHeader.IDE = CAN_ID_STD;
    can1_TxHeader.RTR = CAN_RTR_DATA;
    can1_TxHeader.DLC = 0x08;
    can1_TxData[0] = ch[4] >> 8;
    can1_TxData[1] = ch[4];
    can1_TxData[2] = s[0];
    can1_TxData[3] = s[1];
    can1_TxData[4] = 4;
    can1_TxData[5] = 5;
    can1_TxData[6] = 6;
    can1_TxData[7] = 7;
    HAL_CAN_AddTxMessage(&hcan1, &can1_TxHeader, can1_TxData, &send_mail_box);

    can1_TxHeader.StdId = 0x302;
    can1_TxHeader.IDE = CAN_ID_STD;
    can1_TxHeader.RTR = CAN_RTR_DATA;
    can1_TxHeader.DLC = 0x08;
    can1_TxData[0] = (ins_yaw >> 8) & 0xFF;
    can1_TxData[1] = ins_yaw & 0xFF;
    can1_TxData[2] = (ins_pitch >> 8) & 0xFF;
    can1_TxData[3] = ins_pitch & 0xFF;
    can1_TxData[4] = (ins_roll >> 8) & 0xFF;
    can1_TxData[5] = ins_roll & 0xFF;
    can1_TxData[6] = (yaw_data >> 8) & 0xFF;
    can1_TxData[7] = yaw_data & 0xFF;
    HAL_CAN_AddTxMessage(&hcan1, &can1_TxHeader, can1_TxData, &send_mail_box);
}





//can发送
void send_ins_data_to_a(int32_t ins_yaw, int32_t ins_pitch, int32_t ins_roll,int32_t yaw_data)
{
    CAN_TxHeaderTypeDef txHeader;
    
    uint32_t send_mail_box;
    uint8_t tx_data[8];
    txHeader.StdId = 0x302;
    txHeader.ExtId = 0;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_STD;
    txHeader.DLC = 8;
    ins_yaw += 180;
    ins_pitch += 180;
    ins_roll += 180;

    tx_data[0] = (ins_yaw >> 8) & 0xFF;
    tx_data[1] = ins_yaw & 0xFF;
    tx_data[2] = (ins_pitch >> 8) & 0xFF;
    tx_data[3] = ins_pitch & 0xFF;
    tx_data[4] = (ins_roll >> 8) & 0xFF;
    tx_data[5] = ins_roll & 0xFF;
    tx_data[6] = (yaw_data >> 8) & 0xFF;
    tx_data[7] = yaw_data & 0xFF;
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, tx_data, &send_mail_box);
    // osDelay(10);
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
      // 发送失败
      Error_Handler();
    }
}

