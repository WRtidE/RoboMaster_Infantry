#include "drv_usart.h"
#include  "main.h"

#include  "drv_can.h"
#include  "judge.h"
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
#define USART3_RX_DATA_FRAME_LEN	(18u)	// 串口3数据帧长度
#define USART3_RX_BUF_LEN			(USART3_RX_DATA_FRAME_LEN + 6u)	// 串口3接收缓冲区长度
#define USART6_RX_BUF_LEN   (200)
uint8_t usart3_dma_rxbuf[2][USART3_RX_BUF_LEN];
volatile uint8_t judge_dma_buffer[2][USART6_RX_BUF_LEN] ={0}  ;
uint8_t judge_receive_length=0;

supercap_struct supercap_info;
fp32 supercap_Vi;fp32 supercap_Vo;fp32 supercap_Pi;fp32 supercap_Ii;fp32 supercap_Io;fp32 supercap_Ps;
//输入电压（电池电压）   输出电压         输入功率         输入电流         输出电流      参考恒功率值
uint8_t supercap_temp[51];

void USART3_Init(void)
{
//	__HAL_UART_CLEAR_IDLEFLAG(&huart3);       //清除空闲中断标志位，，防止开启中断时立马进入中断
//	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); //使能空闲中断
//	
//	// Enable the DMA transfer for the receiver request
//	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);	//将串口对应的DMA打开
//	
//	DMAEx_MultiBufferStart_NoIT(huart3.hdmarx, \
//							    (uint32_t)&huart3.Instance->DR, \
//							    (uint32_t)usart3_dma_rxbuf[0], \
//							    (uint32_t)usart3_dma_rxbuf[1], \
//							    USART3_RX_DATA_FRAME_LEN);     //开启DMA双缓冲模式
}

void DRV_USART3_IRQHandler(UART_HandleTypeDef *huart)  //在stm32f4xx_it.c文件USART3_IRQHandler调用
{
    // 判断是否为空闲中断
//	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
//		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
//	{
//		uart_rx_idle_callback(huart); //空闲中断回调函数
//	}
}

void USART6_Init(void)     //开启空闲中断，配置DMA相关参数，使能DMA接收
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);    //先清楚空闲中断标志位，防止开启中断时立马进入中断
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//使能空闲中断

	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR); //将串口对应的DMA打开
	DMAEx_MultiBufferStart_NoIT(huart6.hdmarx, \
							    (uint32_t)&huart6.Instance->DR, \
							    (uint32_t)judge_dma_buffer[0], \
							    (uint32_t)judge_dma_buffer[1], \
							    USART6_RX_BUF_LEN);  
}

void DRV_USART6_IRQHandler(UART_HandleTypeDef *huart)  //在stm32f4xx_it.c文件USART6_IRQHandler调用   
{
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))  //判断是否为空闲中断
	{
		uart_rx_idle_callback(huart);
	}
}




static void uart_rx_idle_callback(UART_HandleTypeDef* huart) 
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);	
	/* handle received data in idle interrupt */
//		 if(huart == &huart3)
//	{
//		/* clear DMA transfer complete flag */
//		__HAL_DMA_DISABLE(huart->hdmarx);

//		/* handle dbus data dbus_buf from DMA */
//		//uint32_t status = taskENTER_CRITICAL_FROM_ISR();
//		if ((USART3_RX_BUF_LEN - huart->hdmarx->Instance->NDTR) == USART3_RX_DATA_FRAME_LEN)
//		{
//			if(huart->hdmarx->Instance->CR & DMA_SxCR_CT)
//				huart->hdmarx->XferM1CpltCallback(huart->hdmarx);
//			else
//				huart->hdmarx->XferCpltCallback(huart->hdmarx);
//		}
//	

//		/* restart dma transmission */
//		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART3_RX_BUF_LEN);
//		__HAL_DMA_ENABLE(huart->hdmarx);	  
//	}
//  
  
    if(huart == &huart6)
	{
		__HAL_DMA_DISABLE(huart->hdmarx);
		judge_receive_length = USART6_RX_BUF_LEN - huart->hdmarx->Instance->NDTR;
	
		if(huart->hdmarx->Instance->CR & DMA_SxCR_CT)
			huart->hdmarx->XferM1CpltCallback(huart->hdmarx);
		else
			huart->hdmarx->XferCpltCallback(huart->hdmarx);
		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART6_RX_BUF_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);	  
	}
}



static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{

//		if(hdma== huart3.hdmarx)
//		{
//				hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT);	 // 将当前目标内存设置为Memory1
//				USART3_rxDataHandler(usart3_dma_rxbuf[0]);
//		}
    if(hdma == huart6.hdmarx)
		{
			hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT);	 // 将当前目标内存设置为Memory1
			JUDGE_Receive(judge_dma_buffer[0],judge_receive_length);
		}

}


static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	
//	if(hdma== huart3.hdmarx)
//	{
//		hdma->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT);	 // 将当前目标内存设置为Memory0
//		USART3_rxDataHandler(usart3_dma_rxbuf[1]);
//	}
	
 if(hdma == huart6.hdmarx)
	{
		hdma->Instance->CR &=~ (uint32_t)(DMA_SxCR_CT);	 // 将当前目标内存设置为Memory0
		JUDGE_Receive(judge_dma_buffer[1],judge_receive_length);
	}
}


static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength)//HAL库stm32f4xx_hal_dma_ex.c中有个类似函数HAL_DMAEx_MultiBufferStart_IT,这里做了修改，关闭了DMA的中断
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


void supercap(uint8_t a)
{
//	uint32_t CAN_TX_MAILBOX01;
//	CAN_TxHeaderTypeDef tx_header;

//		uint8_t power[2];
//	tx_header.StdId = 0x210;
//	tx_header.IDE   = CAN_ID_STD;//标准帧
//	tx_header.RTR   = CAN_RTR_DATA;//数据帧
//	
//	tx_header.DLC   = 2;		//发送数据长度（字节）
//		power[0] = a >> 8;
//		power[1] = a;
//		HAL_CAN_AddTxMessage(&hcan2, &tx_header, power,(uint32_t*)CAN_TX_MAILBOX0);
//	//if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, power,(uint32_t*)CAN_TX_MAILBOX0) == HAL_OK)
//	//{HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);}
	switch(a){
    case 50:
		   HAL_UART_Transmit_IT(&huart1,"P050P",5);
       break; 
    case 60  :
        HAL_UART_Transmit_IT(&huart1,"P060P",5);
       break; 
    case 70  :
			HAL_UART_Transmit_IT(&huart1,"P070P",5);
       break; 
    default : 
       HAL_UART_Transmit_IT(&huart1,"P050P",5);
}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart = &huart1)
	{
		uint8_t i;
		uint8_t j;
		for(i = 0;i<50;i++)
		{
			if(supercap_buffer[i] == 86 && supercap_buffer[i+1] == 105)
			{
				j = i;
				break;
			}
		}
			for (i = 0;i<50;i++)
			{
				supercap_temp[i] = supercap_buffer[j];
				j++;
			}
			supercap_info.Vi = (supercap_temp[3]-48) + (supercap_temp[5]-48)/10.0 + (supercap_temp[6]-48)/100.0;//对supercap_buffer进行处理
			supercap_info.Vo = (supercap_temp[11]-48) + (supercap_temp[13]-48)/10.0 + (supercap_temp[14]-48)/100.0;
			supercap_info.Pi = (supercap_temp[19]-48) + (supercap_temp[21]-48)/10.0 + (supercap_temp[22]-48)/100.0;
			supercap_info.Ii = (supercap_temp[27]-48) + (supercap_temp[29]-48)/10.0 + (supercap_temp[30]-48)/100.0;
			supercap_info.Io = (supercap_temp[35]-48) + (supercap_temp[37]-48)/10.0 + (supercap_temp[38]-48)/100.0;
			supercap_info.Ps = (supercap_temp[43]-48) + (supercap_temp[45]-48)/10.0 + (supercap_temp[46]-48)/100.0;
			HAL_UART_Receive_IT(&huart1,supercap_buffer,100);//重新开启接收中断
	}
}

 


