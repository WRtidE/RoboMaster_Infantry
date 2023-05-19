/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//结构体
typedef struct
{
    uint16_t can_id;		//ID号
    int16_t  set_voltage;		//发送信息
    uint16_t rotor_angle;		//现在的角度
    int16_t  rotor_speed;		//现在的转速
    int16_t  torque_current;		//实际转矩电流
    uint8_t  temp;		//电机温度
}moto_info_t;


//宏定义
#define MOTOR_MAX_NUM 7		//最大数据字节数
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))		//越界则赋边界值
#define FEEDBACK_ID_BASE      0x201
#define FEEDBACK_ID_BASE_6020 0x205
#define CAN_CONTROL_ID_BASE   0x200
#define CAN_CONTROL_ID_EXTEND 0x1ff
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
//全局变量
extern uint16_t can_cnt_1;
extern uint16_t can_cnt_2;
extern float target_speed[7];//实测最大空载转速320rpm
extern float target_speed_can_2[7];
extern moto_info_t motor_info[MOTOR_MAX_NUM];		//赋予最大的7个字节
extern moto_info_t motor_info_can_2[MOTOR_MAX_NUM];
extern uint8_t can_flag;
extern double step; 
extern double r;
extern double target_v;
extern int16_t target_int1;
extern int16_t target_int2;//用于叠加旋转和直行
extern double target_curl;
extern float yuntai_step;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//拨盘
extern float time;
extern float time_count;
extern uint8_t flag_shoot;
extern float round_shoot;
extern float down;
extern float up;
//定时器声明
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
//Yaw轴
#define yaw_front 4096
#define yaw_L 30.0f
#define tyro 3.1415f*7.5f
extern int16_t target_angle;
extern int16_t err_angle;
extern int16_t max_yaw_speed;
extern float small;
extern float angle_limit;

extern uint8_t rx_data[8];
extern uint8_t supercap_buffer[100];
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IST8310_RST_Pin GPIO_PIN_6
#define IST8310_RST_GPIO_Port GPIOG
#define IST8310_DRDY_Pin GPIO_PIN_3
#define IST8310_DRDY_GPIO_Port GPIOG
#define IST8310_DRDY_EXTI_IRQn EXTI3_IRQn
#define CS1_Accel_Pin GPIO_PIN_4
#define CS1_Accel_GPIO_Port GPIOA
#define INT1_Accel_Pin GPIO_PIN_4
#define INT1_Accel_GPIO_Port GPIOC
#define INT1_Accel_EXTI_IRQn EXTI4_IRQn
#define INT1_Gyro_Pin GPIO_PIN_5
#define INT1_Gyro_GPIO_Port GPIOC
#define INT1_Gyro_EXTI_IRQn EXTI9_5_IRQn
#define CS1_Gyro_Pin GPIO_PIN_0
#define CS1_Gyro_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
