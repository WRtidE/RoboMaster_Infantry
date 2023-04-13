/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷遥锟斤拷锟斤拷锟斤拷通锟斤拷锟斤拷锟斤拷SBUS锟斤拷协锟介传锟戒，锟斤拷锟斤拷DMA锟斤拷锟戒方式锟斤拷约CPU
  *             锟斤拷源锟斤拷锟斤拷锟矫达拷锟节匡拷锟斤拷锟叫讹拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷同时锟结供一些锟斤拷锟斤拷锟斤拷锟斤拷DMA锟斤拷锟斤拷锟斤拷
  *             锟侥凤拷式锟斤拷证锟饺诧拷蔚锟斤拷榷锟斤拷浴锟?
  * @note       锟斤拷锟斤拷锟斤拷锟斤拷通锟斤拷锟斤拷锟斤拷锟叫讹拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷freeRTOS锟斤拷锟斤拷
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 锟斤拷锟?
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"
#include "motor.h"
#include "main.h"

#define BUFFER_SIZE 100
extern UART_HandleTypeDef huart1;
volatile uint8_t rx_len_uart1 = 0;  //接收一帧数据的长度
volatile uint8_t recv_end_flag_uart1 = 0; //一帧数据接收完成标志
uint8_t rx_buffer[100]={0};  //接收数据缓存数组
int16_t Yaw_minipc;
int16_t Pitch_minipc;
void Get_minipc();
void remote_data_read(uint8_t rx_buffer[]);


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;



/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥锟斤拷锟斤拷协锟斤拷锟斤拷锟?
  * @param[in]      sbus_buf: 原锟斤拷锟斤拷锟斤拷指锟斤拷
  * @param[out]     rc_ctrl: 遥锟斤拷锟斤拷锟斤拷锟斤拷指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//遥锟斤拷锟斤拷锟斤拷锟狡憋拷锟斤拷
RC_ctrl_t rc_ctrl;

//receive data, 18 bytes one frame, but set 36 bytes 
//锟斤拷锟斤拷原始锟斤拷锟捷ｏ拷为18锟斤拷锟街节ｏ拷锟斤拷锟斤拷36锟斤拷锟街节筹拷锟饺ｏ拷锟斤拷止DMA锟斤拷锟斤拷越锟斤拷
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥锟斤拷锟斤拷锟斤拷始锟斤拷
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}




void remote_data_read(uint8_t rx_buffer[]);

/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          锟斤拷取遥锟斤拷锟斤拷锟斤拷锟斤拷指锟斤拷
  * @param[in]      none
  * @retval         遥锟斤拷锟斤拷锟斤拷锟斤拷指锟斤拷
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}



void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //锟斤拷取锟斤拷锟斤拷锟斤拷锟捷筹拷锟斤拷,锟斤拷锟斤拷 = 锟借定锟斤拷锟斤拷 - 剩锟洁长锟斤拷
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使锟斤拷DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //锟斤拷取锟斤拷锟斤拷锟斤拷锟捷筹拷锟斤拷,锟斤拷锟斤拷 = 锟借定锟斤拷锟斤拷 - 剩锟洁长锟斤拷
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //锟斤拷锟斤拷锟借定锟斤拷锟捷筹拷锟斤拷
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //锟借定锟斤拷锟斤拷锟斤拷0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使锟斤拷DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //锟斤拷锟斤拷遥锟斤拷锟斤拷锟斤拷锟斤拷
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥锟斤拷锟斤拷协锟斤拷锟斤拷锟?
  * @param[in]      sbus_buf: 原锟斤拷锟斤拷锟斤拷指锟斤拷
  * @param[out]     rc_ctrl: 遥锟斤拷锟斤拷锟斤拷锟斤拷指
  * @retval         none
  */
 
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    // rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    // rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    // rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    // rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    // rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    CAN_rc_forward(rc_ctrl->rc.ch,rc_ctrl->rc.s,0,0,0,Yaw_minipc); //锟斤拷锟脚号凤拷锟斤拷锟斤拷A锟斤拷


}

void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = (id_range == 0) ? (0x200) : (0x1ff); //锟斤拷锟絠d_range==0锟斤拷锟斤拷锟?x1ff,id_range==1锟斤拷锟斤拷锟?x2ff锟斤拷ID锟脚ｏ拷
    tx_header.IDE = CAN_ID_STD;                            //锟斤拷准帧
    tx_header.RTR = CAN_RTR_DATA;                          //锟斤拷锟斤拷帧
    tx_header.DLC = 8;                                     //锟斤拷锟斤拷锟斤拷锟捷筹拷锟饺ｏ拷锟街节ｏ拷

    tx_data[0] = (v1 >> 8) & 0xff; //锟饺凤拷锟竭帮拷位
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
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = (id_range == 0) ? (0x1ff) : (0x2ff); //锟斤拷锟絠d_range==0锟斤拷锟斤拷锟?x1ff,id_range==1锟斤拷锟斤拷锟?x2ff锟斤拷ID锟脚ｏ拷
    tx_header.IDE = CAN_ID_STD;                            //锟斤拷准帧
    tx_header.RTR = CAN_RTR_DATA;                          //锟斤拷锟斤拷帧
    tx_header.DLC = 8;                                     //锟斤拷锟斤拷锟斤拷锟捷筹拷锟饺ｏ拷锟街节ｏ拷

    tx_data[0] = (v1 >> 8) & 0xff; //锟饺凤拷锟竭帮拷位
    tx_data[1] = (v1)&0xff;
    tx_data[2] = (v2 >> 8) & 0xff;
    tx_data[3] = (v2)&0xff;
    tx_data[4] = (v3 >> 8) & 0xff;
    tx_data[5] = (v3)&0xff;
    tx_data[6] = (v4 >> 8) & 0xff;
    tx_data[7] = (v4)&0xff;
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}


void Get_minipc()
{

		if(recv_end_flag_uart1 == 1)  //接收完成标志
		{			
			if(rx_buffer[0] == 0x01)
			{
				remote_data_read(rx_buffer);
			}
			recv_end_flag_uart1 = 0;//清除接收结束标志位
			for(uint8_t i=0;i<rx_len_uart1;i++)
				{
					rx_buffer[i]=0;//清接收缓存
				}
				//memset(rx_buffer,0,rx_len);
			rx_len_uart1 = 0;//清除计数
			HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);//重新打开DMA接收
			
		}
}

void remote_data_read(uint8_t rx_buffer[])
{
	Yaw_minipc = (int)(rx_buffer[1] << 8 | rx_buffer[2]);
	Pitch_minipc = (int)(rx_buffer[3] << 8 | rx_buffer[4]);
	Yaw_minipc = (int)(Yaw_minipc * 100)/32767;
	Pitch_minipc = (int)(Pitch_minipc * 100)/32767;
}

