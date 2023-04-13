#include "judge_usart.h"
#include  "main.h"
#include  "judge.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
volatile uint8_t judge_buffer[200] ={0}  ;
uint16_t length=0;


 void USART3_Init(void)     //���������жϣ�����DMA��ز�����ʹ��DMA����
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);    //����������жϱ�־λ����ֹ�����ж�ʱ��������ж�
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//ʹ�ܿ����ж�
	
	// Enable the DMA transfer for the receiver request
	 SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);	//�����ڶ�Ӧ��DMA��
     

	__HAL_DMA_SET_COUNTER(huart3.hdmarx, 200);   //����DMA������200
	 huart3.hdmarx->Instance->PAR =  (uint32_t)&(huart3.Instance->DR);//����DMAԴ��ַ
	 huart3.hdmarx->Instance->M0AR = (uint32_t)judge_buffer;          //����DMAĿ���ַ

    __HAL_DMA_CLEAR_FLAG (&hdma_usart3_rx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart3_rx));  //�������DMA�жϱ�־λ����ֹ������
    __HAL_DMA_CLEAR_FLAG (&hdma_usart3_rx, __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_usart3_rx));
    __HAL_DMA_CLEAR_FLAG (&hdma_usart3_rx, __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_usart3_rx));
    __HAL_DMA_CLEAR_FLAG (&hdma_usart3_rx, __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_usart3_rx));
    __HAL_DMA_CLEAR_FLAG (&hdma_usart3_rx, __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_usart3_rx));
	  __HAL_DMA_ENABLE(huart3.hdmarx);         //����DMA����
}

   void DRV_USART3_IRQHandler(void)  //��stm32f4xx_it.c�ļ�USART3_IRQHandler����   
{
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE)) //�ж��Ƿ�Ϊ�����ж�
	{
		 uint8_t receive_length;
		__HAL_DMA_DISABLE(&hdma_usart3_rx); //��һ��DMA������һ��������˵
   	__HAL_UART_CLEAR_IDLEFLAG(&huart3);  //��������жϱ�־λ
		receive_length = 200 - hdma_usart3_rx.Instance->NDTR; //���ճ���=�趨����-ʣ�೤��
    
		JUDGE_Receive(judge_buffer,receive_length);//���ݴ����������ϵͳ�ṹ����

		hdma_usart3_rx.Instance->NDTR  =200;//�趨����
		hdma_usart3_rx.Instance->M0AR  =(uint32_t)judge_buffer; //�洢��ַ����



		__HAL_DMA_ENABLE(&hdma_usart3_rx);//����DMA
	}
	
}
