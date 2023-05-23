/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң����������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж���������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�?
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. ���?
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"
#include "main.h"
#include "Can_user.h"
#include "INS_task.h"
#define BUFFER_SIZE 100
extern UART_HandleTypeDef huart1;
volatile uint8_t rx_len_uart1 = 0;  //����һ֡���ݵĳ���
volatile uint8_t recv_end_flag_uart1 = 0; //һ֡���ݽ�����ɱ�־


uint8_t rx_buffer[100]={0};  //�������ݻ�������
int16_t Yaw_minipc;
int16_t Pitch_minipc;
fp32 Yaw_minipc_fp;
fp32 Pitch_minipc_fp;


void Get_minipc();
void remote_data_read(uint8_t rx_buffer[]);
extern ins_data_t ins_data;

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

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern uint16_t angle_1;
extern uint16_t angle_2;
extern uint16_t angle_3;

uint8_t flag = 0;

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����?
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//ң�������Ʊ���
RC_ctrl_t rc_ctrl;

//receive data, 18 bytes one frame, but set 36 bytes 
//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ң������ʼ��
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
  * @brief          ��ȡң��������ָ��
  * @param[in]      none
  * @retval         ң��������ָ��
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}



void USART3_IRQHandler(void)
{
	flag = 2;
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
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
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
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
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
  * @brief          ң����Э�����?
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
 
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
	flag = 6;
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

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
	
		
	r_flag=(sbus_buf[15]&0x01);
	f_flag=(sbus_buf[15]&0x02);
	g_flag=(sbus_buf[15]&0x04);
	z_flag=(sbus_buf[15]&0x08);
	x_flag=(sbus_buf[15]&0x10);
	c_flag=(sbus_buf[15]&0x20);
	v_flag=(sbus_buf[15]&0x40);
	b_flag=(sbus_buf[15]&0x80);
	
	w_flag=(sbus_buf[14]&0x01);
	s_flag=(sbus_buf[14]&0x02);
	a_flag=(sbus_buf[14]&0x04);
	d_flag=(sbus_buf[14]&0x08);
	q_flag=(sbus_buf[14]&0x40);
	e_flag=(sbus_buf[14]&0x80);
	shift_flag=(sbus_buf[14]&0x10);
	ctrl_flag=(sbus_buf[14]&0x20);
	
	can_rc_forward(*rc_ctrl);
}

void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3,int16_t v4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = (id_range == 0) ? (0x200) : (0x1ff); //���id_range==0�����?x1ff,id_range==1�����?x2ff��ID�ţ�
    tx_header.IDE = CAN_ID_STD;                            //��׼֡
    tx_header.RTR = CAN_RTR_DATA;                          //����֡
    tx_header.DLC = 8;                                     //�������ݳ��ȣ��ֽڣ�

    tx_data[0] = (v1 >> 8) & 0xff; //�ȷ��߰�λ
    tx_data[1] = (v1)&0xff;
    tx_data[2] = (v2 >> 8) & 0xff;
    tx_data[3] = (v2)&0xff;
    tx_data[4] = (v3 >> 8) & 0xff;
    tx_data[5] = (v3)&0xff;
    tx_data[6] = (v4 >> 8) & 0xff;
    tx_data[7] = (v4)&0xff;
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}
void set_motor_voltage1(uint8_t id_range, int16_t v1,int16_t v2,int16_t v3,int16_t v4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = (id_range == 0) ? (0x1ff) : (0x2ff); //���id_range==0�����?x1ff,id_range==1�����?x2ff��ID�ţ�
    tx_header.IDE = CAN_ID_STD;                            //��׼֡
    tx_header.RTR = CAN_RTR_DATA;                          //����֡
    tx_header.DLC = 8;                                     //�������ݳ��ȣ��ֽڣ�

    tx_data[0] = (v1 >> 8) & 0xff; //�ȷ��߰�λ
    tx_data[1] = (v1)&0xff;
    tx_data[2] = (v2 >> 8) & 0xff; //�ȷ��߰�λ
    tx_data[3] = (v2)&0xff;
    tx_data[4] = (v3 >> 8) & 0xff; //�ȷ��߰�λ
    tx_data[5] = (v3)&0xff;
    tx_data[6] = (v4 >> 8) & 0xff; //�ȷ��߰�λ
    tx_data[7] = (v4)&0xff;
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}


void Get_minipc()

{

		if(recv_end_flag_uart1 == 1)  //������ɱ�־
		{			
			if(rx_buffer[0] == 0x01)
			{
				remote_data_read(rx_buffer);
			}
			recv_end_flag_uart1 = 0;//������ս�����־λ
			for(uint8_t i=0;i<rx_len_uart1;i++)
				{
					rx_buffer[i]=0;//����ջ���
				}
				//memset(rx_buffer,0,rx_len);
			rx_len_uart1 = 0;//�������
			HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);//���´�DMA����
			
		}
}

void remote_data_read(uint8_t rx_buffer[])
{
	Yaw_minipc = (int)(rx_buffer[1] << 8 | rx_buffer[2]);
	Pitch_minipc = (int)(rx_buffer[3] << 8 | rx_buffer[4]);
	Yaw_minipc_fp = (float)(Yaw_minipc * 100)/32767; 
	Pitch_minipc_fp = (float)(Pitch_minipc * 100)/32767;
//	Yaw_minipc = (int)(Yaw_minipc * 100)/32767;
//	Pitch_minipc = (int)(Pitch_minipc * 100)/32767;
}

