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
#include "struct_typedef.h"
#include "arm_math.h"
#include "stdbool.h"	
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
//�ṹ��
typedef struct
{
    uint16_t can_id;		//ID��
    int16_t  set_voltage;		//������Ϣ
    uint16_t rotor_angle;		//���ڵĽǶ�
    int16_t  rotor_speed;		//���ڵ�ת��
    int16_t  torque_current;		//ʵ��ת�ص���
    uint8_t  temp;		//����¶�
}moto_info_t;

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float ref;      // target value
  float fdb;      // feedback value  
  float err[2];   // error and last error

  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct_t;

//�궨��
#define MOTOR_MAX_NUM 7		//��������ֽ���
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))		//Խ���򸳱߽�ֵ
#define FEEDBACK_ID_BASE      0x201
#define FEEDBACK_ID_BASE_6020 0x205
#define CAN_CONTROL_ID_BASE   0x200
#define CAN_CONTROL_ID_EXTEND 0x1ff
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
//ȫ�ֱ���
extern uint16_t can_cnt_1;
extern uint16_t can_cnt_2;
extern float target_speed[7];//ʵ��������ת��320rpm
extern float target_speed_can_2[7];
extern moto_info_t motor_info[MOTOR_MAX_NUM];		//��������7���ֽ�
extern moto_info_t motor_info_can_2[MOTOR_MAX_NUM];
extern pid_struct_t motor_pid[7];	
extern pid_struct_t motor_pid_can_2[7];	
extern uint8_t can_flag;
extern double step; 
extern double r;
extern double sin_sita;
extern double cos_sita;
extern double target_v;
extern int16_t target_int1;
extern int16_t target_int2;//���ڵ�����ת��ֱ��
extern double target_curl;
extern float yuntai_step;


extern uint32_t shoot_flag ;	

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//��ʱ������
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;



extern pid_struct_t pitch_pid[7];
//Ħ����
extern uint8_t flag_shooting;
extern uint8_t             rx_data[8];
//������
extern fp32 mag[3];
//���ٶȼƺ�������
extern fp32 gyro[3], accel[3], temp;
#define BMI088_ACCEL_3G_SEN 0.0008974358974f    //���������Ҳ��֪��������
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f    //���������Ҳ��֪��������
extern float BMI088_ACCEL_SEN;
extern float BMI088_GYRO_SEN;
extern uint8_t buf_accel[8];
extern uint8_t buf_gyro[8];
extern uint8_t pTxData;
extern uint8_t pRxData;
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern float quat[4];
extern float INS_angle[3];
//���ٶȼƽ���ŷ����
extern float roll_accel;
extern float pitch_accel;

//������ŷ���ǽ���
//extern float gyro_vector[3];
//extern arm_matrix_instance_f32 gyro_matrix;

//��Ԫ��
extern float q0;
extern float q1;
extern float q2;
extern float q3;
extern float T;
extern float halfT;
extern float pitch;
extern float roll;
extern float yaw;
//һЩ��̬���㸨������
extern float norm;
extern float gx, gy, gz;//������
extern float ax, ay, az;//���ٶȼ�
extern float vx, vy, vz, wx, wy, wz;
extern float ex, ey, ez; 
extern float exInt, eyInt, ezInt;
extern float Kp, Ki;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Laser_Pin GPIO_PIN_8
#define Laser_GPIO_Port GPIOC
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
