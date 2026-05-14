#ifndef INS_Task_H
#define INS_Task_H
#include "struct_typedef.h"
#include "tim.h"
#include "spi.h"
 #include "math.h"
#include "cmsis_os.h"
 #include  "pid_imu.h" 
 
 extern SPI_HandleTypeDef hspi1;
  extern I2C_HandleTypeDef hi2c3;
  extern TIM_HandleTypeDef htim10;
//  fp32 INS_gyro[3] ;
// fp32 INS_accel[3] ;
// fp32 INS_mag[3] ;
// fp32 INS_quat[4] ;

  #define ACCEL_L  HAL_GPIO_WritePin(GPIOA, CS1_Accel_Pin, GPIO_PIN_RESET)
  #define ACCEL_H  HAL_GPIO_WritePin(GPIOA, CS1_Accel_Pin, GPIO_PIN_SET)
  #define GYRO_L   HAL_GPIO_WritePin(GPIOB,CS1_Gyro_Pin, GPIO_PIN_RESET)
  #define GYRO_H   HAL_GPIO_WritePin(GPIOB,CS1_Gyro_Pin, GPIO_PIN_SET)
  
  
  #define sampleFreq	1000.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

typedef struct INS_DATA
{
    fp32 accel_offset[3];
    fp32 gyro_offset[3];
    fp32 accel[3];
    fp32 temp;
    fp32 gyro[3];
    fp32 angle[3];
    fp32 INS_quat[4];  
} ins_data_t;

typedef struct ist8310_real_data_t
{
  uint8_t status;
  float mag[3];
} ist8310_real_data_t;



 void INS_Task(void const *pvParameters) ;
void imu_temp_control(fp32 temp);
void BMI088_accel_write_single (uint8_t txbuf,uint8_t data);
void BMI088_gyro_write_single (uint8_t txbuf,uint8_t data);
uint8_t  BMIO88_accel_read_single(uint8_t reg);
uint8_t  BMIO88_gyro_read_single(uint8_t reg);
void BMI088_accel_init(void);
void BMI088_gyro_init(void);
void BMI088_accel_read_muli(uint8_t reg,uint8_t* buf,uint8_t len);
void BMI088_gyro_read_muli(uint8_t reg,uint8_t* buf);
void BMI088_read(fp32 *accel,fp32 *gyro,fp32* temp);
void IMU_offset_cali(void);


void ist8310_init(void);
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
void ist8310_read_mag(fp32* mag);
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);



void AHRS_init(ins_data_t * ins_data);


float invSqrt(float x);
void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll);



#endif