#ifndef __DRV_USART_H
#define __DRV_USART_H

#include "main.h"
#include "struct_typedef.h"

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
void DRV_USART3_IRQHandler(UART_HandleTypeDef *huart);
void DRV_USART6_IRQHandler(UART_HandleTypeDef *huart);
void USART3_Init(void);
void USART6_Init(void);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength);
typedef struct
{
    
    fp32 Vi;
    fp32 Vo;
    fp32 Pi;
    fp32 Ii;  
    fp32 Io; 
    fp32 Ps;

} supercap_struct;

extern void supercap_uart_send(uint8_t a);
extern supercap_struct supercap_info;
#endif