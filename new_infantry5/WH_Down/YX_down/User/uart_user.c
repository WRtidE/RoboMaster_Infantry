#include "uart_user.h"

void USART1_IRQHandler(void)//ע�⣬������Ҫ��ԭ�ļ�stm32f4xx_it.c����ͬ�������ļ�ɾȥһ��
{

	uint32_t tmp_flag = 0;
	uint32_t temp;
	tmp_flag =__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE); //��ȡIDLE��־λ
	if((tmp_flag != RESET))//idle��־����λ
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);//�����־λ
		//temp = huart1.Instance->SR;  //���״̬�Ĵ���SR,��ȡSR�Ĵ�������ʵ�����SR�Ĵ����Ĺ���
		//temp = huart1.Instance->DR; //��ȡ���ݼĴ����е�����
		//������������Ǿ��Ч
		HAL_UART_DMAStop(&huart1); //ֹͣDMA
		temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);// ��ȡDMA��δ��������ݸ���   
		//temp  = hdma_usart1_rx.Instance->NDTR;//��ȡNDTR�Ĵ��� ��ȡDMA��δ��������ݸ�����
		//���������Ǿ��Ч
		rx_len =  100 - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
		recv_end_flag = 1;	// ������ɱ�־λ��1	
	 }
  HAL_UART_IRQHandler(&huart1);

}