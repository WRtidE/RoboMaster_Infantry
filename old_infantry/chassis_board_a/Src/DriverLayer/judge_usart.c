#include "judge_usart.h"
#include  "main.h"
#include  "judge.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
volatile uint8_t judge_buffer[200] ={0}  ;
uint16_t length=0;


 void USART3_Init(void)     //开启空闲中断，配置DMA相关参数，使能DMA接收
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);    //先清楚空闲中断标志位，防止开启中断时立马进入中断
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//使能空闲中断
	
	// Enable the DMA transfer for the receiver request
	 SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);	//将串口对应的DMA打开
     

	__HAL_DMA_SET_COUNTER(huart3.hdmarx, 200);   //设置DMA接收量200
	 huart3.hdmarx->Instance->PAR =  (uint32_t)&(huart3.Instance->DR);//设置DMA源地址
	 huart3.hdmarx->Instance->M0AR = (uint32_t)judge_buffer;          //设置DMA目标地址

    __HAL_DMA_CLEAR_FLAG (&hdma_usart3_rx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart3_rx));  //清除各种DMA中断标志位，防止出问题
    __HAL_DMA_CLEAR_FLAG (&hdma_usart3_rx, __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_usart3_rx));
    __HAL_DMA_CLEAR_FLAG (&hdma_usart3_rx, __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_usart3_rx));
    __HAL_DMA_CLEAR_FLAG (&hdma_usart3_rx, __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_usart3_rx));
    __HAL_DMA_CLEAR_FLAG (&hdma_usart3_rx, __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_usart3_rx));
	  __HAL_DMA_ENABLE(huart3.hdmarx);         //开启DMA接收
}

   void DRV_USART3_IRQHandler(void)  //在stm32f4xx_it.c文件USART3_IRQHandler调用   
{
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE)) //判断是否为空闲中断
	{
		 uint8_t receive_length;
		__HAL_DMA_DISABLE(&hdma_usart3_rx); //关一下DMA，处理一下数据再说
   	__HAL_UART_CLEAR_IDLEFLAG(&huart3);  //清楚空闲中断标志位
		receive_length = 200 - hdma_usart3_rx.Instance->NDTR; //接收长度=设定长度-剩余长度
    
		JUDGE_Receive(judge_buffer,receive_length);//数据处理，读入裁判系统结构体中

		hdma_usart3_rx.Instance->NDTR  =200;//设定长度
		hdma_usart3_rx.Instance->M0AR  =(uint32_t)judge_buffer; //存储地址设置



		__HAL_DMA_ENABLE(&hdma_usart3_rx);//开启DMA
	}
	
}
