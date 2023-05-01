#include "judge_usart.h"
#include  "main.h"
#include  "judge.h"


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
#define USART3_RX_BUF_LEN   (200)
volatile uint8_t judge_dma_buffer[2][USART3_RX_BUF_LEN] ={0}  ;
uint8_t judge_receive_length=0;
uint8_t flag = 0 ;


void USART3_Init(void)     //���������жϣ�����DMA��ز�����ʹ��DMA����
{
	flag = 6;
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);    //����������жϱ�־λ����ֹ�����ж�ʱ��������ж�
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//ʹ�ܿ����ж�

	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR); //�����ڶ�Ӧ��DMA��
	
	//����STM32΢��������DMA��ֱ���ڴ���ʣ����䣬�Դ�UART3�������ݡ��ú���ʹ����˫������ģʽ��
	DMAEx_MultiBufferStart_NoIT(huart3.hdmarx,  \
							    (uint32_t)&huart3.Instance->DR, \
							    (uint32_t)judge_dma_buffer[0], \
							    (uint32_t)judge_dma_buffer[1], \
							    USART3_RX_BUF_LEN);   //����DMA˫����ģʽ
}


void DRV_USART3_IRQHandler(UART_HandleTypeDef *huart)  //��stm32f4xx_it.c�ļ�USART3_IRQHandler����   
{
	flag = 1;
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))  //�ж��Ƿ�Ϊ�����ж�
	{
		uart_rx_idle_callback(huart);
	}
}




static void uart_rx_idle_callback(UART_HandleTypeDef* huart) 
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);	//��������жϱ�־λ
	/* handle received data in idle interrupt */
    if(huart == &huart3)
	{
		__HAL_DMA_DISABLE(huart->hdmarx);  //�ر�DMA�����ݽ�����ɣ���ʼ�������ݣ�
		judge_receive_length = USART3_RX_BUF_LEN - huart->hdmarx->Instance->NDTR;

		//˫����DMA
		if(huart->hdmarx->Instance->CR & DMA_SxCR_CT) //�ж��Ƿ�Ϊ�ڶ����ڴ����д���
			huart->hdmarx->XferM1CpltCallback(huart->hdmarx);
		else
			huart->hdmarx->XferCpltCallback(huart->hdmarx);
		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART3_RX_BUF_LEN);

		__HAL_DMA_ENABLE(huart->hdmarx);	  
	}
}

static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
		if(hdma == huart3.hdmarx)
		{
			hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT);	 // ����ǰĿ���ڴ�����ΪMemory1
			JUDGE_Receive(judge_dma_buffer[0],judge_receive_length);
		}

}


static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	if(hdma == huart3.hdmarx)
	{
		hdma->Instance->CR &=~ (uint32_t)(DMA_SxCR_CT);	 // ����ǰĿ���ڴ�����ΪMemory0
		JUDGE_Receive(judge_dma_buffer[1],judge_receive_length);
	}
}


static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength)//HAL��stm32f4xx_hal_dma_ex.c���и����ƺ���HAL_DMAEx_MultiBufferStart_IT,���������޸ģ��ر���DMA���ж�
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Memory-to-memory transfer not supported in double buffering mode */
    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
		hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
		return HAL_ERROR;
    }   

	/* Set the UART DMA transfer complete callback */
	/* Current memory buffer used is Memory 1 callback */
	hdma->XferCpltCallback   = dma_m0_rxcplt_callback;
	/* Current memory buffer used is Memory 0 callback */
	hdma->XferM1CpltCallback = dma_m1_rxcplt_callback;	

	/* Check callback functions */
	if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback))
	{
	hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
	return HAL_ERROR;
	}
	
	/* Process locked */
	__HAL_LOCK(hdma);
	
	if(HAL_DMA_STATE_READY == hdma->State)
	{	
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Enable the Double buffer mode */
		hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;

		/* Configure DMA Stream destination address */
		hdma->Instance->M1AR = SecondMemAddress;		

		/* Configure DMA Stream data length */
		hdma->Instance->NDTR = DataLength;		
		
		/* Peripheral to Memory */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{   
			/* Configure DMA Stream destination address */
			hdma->Instance->PAR = DstAddress;

			/* Configure DMA Stream source address */
			hdma->Instance->M0AR = SrcAddress;
		}
		/* Memory to Peripheral */
		else
		{
			/* Configure DMA Stream source address */
			hdma->Instance->PAR = SrcAddress;

			/* Configure DMA Stream destination address */
			hdma->Instance->M0AR = DstAddress;
		}		
		
		/* Clear TC flags */
		__HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
		/* Enable TC interrupts*/
//		hdma->Instance->CR  |= DMA_IT_TC;
		
		/* Enable the peripheral */
		__HAL_DMA_ENABLE(hdma); 
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);	  

		/* Return error status */
		status = HAL_BUSY;		
	}
	/* Process unlocked */
	__HAL_UNLOCK(hdma);

	return status; 	
}




