#include  "drv_can.h"
#include "INS_task.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl;
uint16_t can_cnt_1=0;
extern motor_info_t  motor_info_chassis[7];
extern uint16_t Up_ins_yaw;
float powerdata[4];
uint16_t pPowerdata[8];
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

uint16_t setpower = 5500;
int16_t mouse_x; 
ins_data_t ins_data1;
fp32 yaw_data;
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
void CAN1_Init(void)
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
    can_filter.SlaveStartFilterBank  = 14;          
   
    HAL_CAN_ConfigFilter(&hcan1, &can_filter);        // init can filter
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);		//使能can的FIFO0中断
    HAL_CAN_Start(&hcan1);//启动can1

}

void CAN2_Init( void )
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
    can_filter.SlaveStartFilterBank  = 14;         
   
    HAL_CAN_ConfigFilter(&hcan2, &can_filter);        // init can filter
   	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	  HAL_CAN_Start(&hcan2);//启动can2
}


//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//接受中断回调函数
//{
//  CAN_RxHeaderTypeDef rx_header;

//  if(hcan->Instance == CAN1)
//  {
//     uint8_t rx_data[8];
//		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can1 data
//		if(rx_header.StdId==0x55)//上C向下C传IMU数据
//		{	
//				if(rx_data[0] == 8)	//校验位
//				{
//					Up_ins_yaw = rx_data[1] | (rx_data[2] << 8);			
//				}
//		}
//  }
//	 if(hcan->Instance == CAN2)
//  {		uint8_t             rx_data[8];
//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can2 data
//		if ((rx_header.StdId >= 0x201)//201-207
//   && (rx_header.StdId <  0x208))                  // 判断标识符，标识符为0x200+ID
//  {
//    uint8_t index = rx_header.StdId - 0x201;                  // get motor index by can_id
//     motor_info_chassis[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
//     motor_info_chassis[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
//     motor_info_chassis[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
//     motor_info_chassis[index].temp           =   rx_data[6];
//		if(index==0)
//		{can_cnt_1 ++;}
//  }
//	if(rx_header.StdId==0x211)
//			{
//				
//				extern float powerdata[4];
//				uint16_t *pPowerdata = (uint16_t *)rx_data;

//				powerdata[0] = (float)pPowerdata[0]/100.f;//输入电压
//				powerdata[1] = (float)pPowerdata[1]/100.f;//电容电压
//				powerdata[2] =(float)pPowerdata[2]/100.f;//输入电流
//				powerdata[3] = (float)pPowerdata[3]/100.f;//P
//				
//				
//			
//  }}

//}
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
            ins_data1.angle[0] = (can1_RxData[0] << 8) | can1_RxData[1];
            ins_data1.angle[1] = (can1_RxData[2] << 8) | can1_RxData[3];
            ins_data1.angle[2] = (can1_RxData[4] << 8) | can1_RxData[5];
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
            motor_info_chassis[index].rotor_angle = ((rx_data[0] << 8) | rx_data[1]);
            motor_info_chassis[index].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
            motor_info_chassis[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
            motor_info_chassis[index].temp = rx_data[6];
        }if ((rx_header.StdId >= FEEDBACK_ID_BASE_6020)                    // 205-211,注意把ID调成大于3,不然就会和读取3508的函数产生冲突
            && (rx_header.StdId < FEEDBACK_ID_BASE_6020 + MOTOR_MAX_NUM)) // 判断标识符，标识符为0x204+ID
        {
            uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE_6020; // get motor index by can_id
            motor_info_chassis[index].rotor_angle = ((rx_data[0] << 8) | rx_data[1]);
            motor_info_chassis[index].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
            motor_info_chassis[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
            motor_info_chassis[index].temp = rx_data[6];
        }
        
    }

    // HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
}

//void can_remote(uint8_t sbus_buf[],uint8_t can_send_id)//调用can来发送遥控器数据
//{
//  CAN_TxHeaderTypeDef tx_header;
//    
//  tx_header.StdId = can_send_id;//如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
//  tx_header.IDE   = CAN_ID_STD;//标准帧
//  tx_header.RTR   = CAN_RTR_DATA;//数据帧
//  tx_header.DLC   = 8;		//发送数据长度（字节）

//  HAL_CAN_AddTxMessage(&hcan1, &tx_header, sbus_buf,(uint32_t*)CAN_TX_MAILBOX0);
//}

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


void supercap(a)
{
	uint32_t CAN_TX_MAILBOX01;
	CAN_TxHeaderTypeDef tx_header;

    uint8_t power[2];
  tx_header.StdId = 0x210;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
	
  tx_header.DLC   = 2;		//发送数据长度（字节）
		power[0] = a >> 8;
		power[1] = a;
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, power,(uint32_t*)CAN_TX_MAILBOX0);
	//if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, power,(uint32_t*)CAN_TX_MAILBOX0) == HAL_OK)
	//{HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);}
	
}

void judge_data_send(int16_t heat, int16_t heat_limit, int16_t level, int16_t v4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = 0x305; 
    tx_header.IDE = CAN_ID_STD;                            //标准帧
    tx_header.RTR = CAN_RTR_DATA;                          //数据帧
    tx_header.DLC = 8;                                     //发送数据长度（字节）

    tx_data[0] = (heat >> 8) & 0xff; //枪口热量
    tx_data[1] = (heat)&0xff;
    tx_data[2] = (heat_limit >> 8) & 0xff; //枪口热量上限
    tx_data[3] = (heat_limit)&0xff;
    tx_data[4] = (level)&0xff; //子弹射速 单位m/s
    tx_data[5] = 0;
    tx_data[6] = (v4 >> 8) & 0xff; 
    tx_data[7] = (v4)&0xff;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}