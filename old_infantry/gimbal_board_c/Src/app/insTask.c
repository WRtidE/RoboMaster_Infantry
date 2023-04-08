 #include "INSTask.h"

float Q_angle=0.005;//角度方差
	float Q_gyro_bias=0.004;//角速度方差
	float R_measure=0.3;//测量噪声
float dt = 0.001;
float p[2][2]= {{0,0},{0,0}};
static const fp32 imu_temp_PID[3] = {1600, 0.2, 0};	//PID的3个值
 pid_type_def  imu_temp_pid;
fp32 first_temperate;
ins_data_t ins_data;
volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
size_t a;


//卡尔曼滤波器
void Kalman_Fliter(float *gx, float *ax)
{
	float ax1 = *ax;//方便后续残差计算的角速度初始值
	
	
	float Q_bias=0;//角度偏差
	float k1;//angle卡尔曼增益
	float k2;//偏移卡尔曼增益
	//先验估计
	*ax = *ax - Q_bias + *gx *dt;
	Q_bias = Q_bias;
	//预测协方差
	p[0][0]= p[0][0]+(p[1][1]*dt+Q_angle-p[0][1]-p[1][0]) *dt;
	p[0][1]= p[0][1]-p[1][1];
	p[1][0]= p[1][0]-p[1][1];
	p[1][1]=p[1][1] + Q_gyro_bias *dt;
	//卡尔曼增益
	k1 = p[0][0]/(p[0][0] + R_measure);
	k2 = p[1][0]/(p[0][0] + R_measure);
	//计算残差
	float z = ax1;
	float y = z - *ax;
	//状态更新
	*ax = *ax + k1 * y;//角度
	Q_bias = Q_bias + k2 * y;//偏移
	//协方差矩阵更新
	p[0][0]=p[0][0]*(1-k1);
	p[0][1]=p[0][1]*(1-k1);
	p[1][0]=p[1][0]-p[0][0] * k2;
	p[1][1]=p[1][1]-p[0][1] * k2;
	
	

	
}
  void INS_Task(void const *pvParameters)   
{
  
     osDelay(7);
     BMI088_accel_init();		//加速度计初始化
     BMI088_gyro_init();		//陀螺仪初始化
     ist8310_init();		//磁力计初始化（就是一些根据手册写的读写函数）
     IMU_offset_cali();
    AHRS_init(& ins_data);	//初始化四元数的值
     PID_init(&imu_temp_pid, 0, imu_temp_PID, 4500, 4400);//后两位是P和I的MAX限制值
  
    while(1)
    {
       BMI088_read(ins_data.accel, ins_data.gyro, &ins_data.temp);
       imu_temp_control(ins_data.temp);
			Kalman_Fliter(&ins_data.gyro[0], &ins_data.accel[0]);
			Kalman_Fliter(&ins_data.gyro[1], &ins_data.accel[1]);
			Kalman_Fliter(&ins_data.gyro[2], &ins_data.accel[2]);
       MahonyAHRSupdateIMU(ins_data.INS_quat, ins_data.gyro[0], ins_data.gyro[1], ins_data.gyro[2], ins_data.accel[0], ins_data.accel[1], ins_data.accel[2]); 
       get_angle(ins_data.INS_quat, &ins_data.angle[0], &ins_data.angle[1], &ins_data.angle[2]);
       a= uxTaskGetStackHighWaterMark( NULL );
       osDelay(1);
    }
  }



  






#define IMU_temp_PWM(pwm)    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm)

 void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, 40);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 40)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = 5000 / 2.0f;
            }
        }

        IMU_temp_PWM(5000 - 1);
    }
}




  


  void BMI088_accel_write_single (uint8_t txbuf,uint8_t data)
  {
      ACCEL_L;
      HAL_SPI_Transmit_DMA(&hspi1,&txbuf,1);
      while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    
      HAL_SPI_Transmit_DMA(&hspi1,&data,1);
      while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX){}; 
      osDelay(1);   
      ACCEL_H;
  }


     void BMI088_gyro_write_single (uint8_t txbuf,uint8_t data)
  {
        GYRO_L;
        HAL_SPI_Transmit_DMA(&hspi1,&txbuf,1);
        while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    
        HAL_SPI_Transmit_DMA(&hspi1,&data,1);
        while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);  
        GYRO_H; 
  }




    uint8_t BMIO88_accel_read_single(uint8_t reg)
  {
      uint8_t temp;
      ACCEL_L;
      temp=(reg|0x80);
      HAL_SPI_Transmit_DMA(&hspi1,&temp,1);
      while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX){}; 
     	HAL_SPI_Receive_DMA(&hspi1,&temp,1); 
      while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);  
      HAL_SPI_Receive_DMA(&hspi1,&temp,1); 
      while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);
      ACCEL_H;
      return  temp;
  } 

    uint8_t  BMIO88_gyro_read_single(uint8_t reg)
  {
      uint8_t temp;
       GYRO_L;
      temp=(reg|0x80);
      HAL_SPI_Transmit_DMA(&hspi1,&temp,1);
      while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX); 
      HAL_SPI_Receive_DMA(&hspi1,&temp,1); 
      while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);
      GYRO_H;
      return temp;
  }

     void BMI088_accel_init(void)
  {
     //taskENTER_CRITICAL();
      BMI088_accel_write_single(0x7E,0xB6);   
       osDelay(10);
      BMI088_accel_write_single(0x7D,0X04); 
       osDelay(10);
      BMI088_accel_write_single(0x7C,0x00);   
       osDelay(10);
      BMI088_accel_write_single(0x41,0x00);    
       osDelay(10);
      BMI088_accel_write_single(0x53,(0x01<<3)|(0X00<<2)|(0X00<<1));
       osDelay(10);
      BMI088_accel_write_single(0x58,(0x01<<2));
       osDelay(10);
    //taskEXIT_CRITICAL();
    
  }


     void BMI088_gyro_init ()
  {
    // taskENTER_CRITICAL();
    BMI088_gyro_write_single(0x14,0xB6);//�???????螺仪软复位，等待30ms
     osDelay(30);
    BMI088_gyro_write_single(0x0F,0x00);//满量�???????2000°每秒
     osDelay(10);
    BMI088_gyro_write_single(0x16,(0x00<<1)|(0x00<<0));
     osDelay(10);
    BMI088_gyro_write_single(0x18,0x01);
     osDelay(10);
  // taskEXIT_CRITICAL();
  }      
  

  
     void BMI088_accel_read_muli(uint8_t reg,uint8_t* buf,uint8_t len)
{
        uint8_t i=0;
        uint8_t pTxData,pRxData;
        
       ACCEL_L; 
        pTxData = (reg|0x80);
        HAL_SPI_Transmit_DMA(&hspi1,&pTxData,1);
        while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
        HAL_SPI_Receive_DMA(&hspi1,&pRxData,1);       
        while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);
        while (i < len)
     {
        HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);    //Bit #16-23���Ĵ���0x12��ֵ��Ȼ���ǼĴ���0x13��0x14��0x15��0x16��0x17��ֵ
    	  while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    //�ȴ�SPI�������???????
    	  buf[i] = pRxData;
        i++;
     }
        ACCEL_H;  
}

  void BMI088_gyro_read_muli(uint8_t reg,uint8_t* buf)
   {
       uint8_t i=0;
     uint8_t pTxData,pRxData;
        GYRO_L;
       pTxData = (reg|0x80);
       HAL_SPI_Transmit_DMA(&hspi1,&pTxData,1);
       while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
       while (i < 6)
      {
        HAL_SPI_Receive(&hspi1, &pRxData, 1, 1000);     
    	  while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);    
    	  buf[i] = pRxData;
        i++;
      }
        GYRO_H;
   }


void BMI088_read(fp32 *accel,fp32 *gyro,fp32* temp)
{
  
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
    BMI088_accel_read_muli(0x12, buf,6);
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * 0.0008974358974f;   //+-3g  0.0008974358974f
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * 0.0008974358974f;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * 0.0008974358974f;
    
      
     
    BMI088_gyro_read_muli(0x02,buf);
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    gyro[0] = bmi088_raw_temp * 0.00106526443603169529841533860381f;  //2000° 0.00106526443603169529841533860381f
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    gyro[1] = bmi088_raw_temp * 0.00106526443603169529841533860381f;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    gyro[2] = bmi088_raw_temp * 0.00106526443603169529841533860381f;
    BMI088_accel_read_muli(0x22, buf, 2);
    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }
    *temp = bmi088_raw_temp * 0.125 + 23.0;

    accel[0]-=ins_data.accel_offset[0];
    accel[1]-=ins_data.accel_offset[1];
    accel[2]-=ins_data.accel_offset[2];
    gyro[0]-=ins_data.gyro_offset[0];
    gyro[1]-=ins_data.gyro_offset[1];
    gyro[2]-=ins_data.gyro_offset[2];
}


  void  IMU_offset_cali()
  {
     int cnt=0;
    fp32  gyro_offsets_sum[3];	//fp就是float类型
    fp32  accel_offsets_sum[3];
    while(cnt <= 3000)
   {
      BMI088_read(ins_data.accel,ins_data.gyro,&ins_data.temp);
      gyro_offsets_sum[0]+=ins_data.gyro[0];
      gyro_offsets_sum[1]+=ins_data.gyro[1];
      gyro_offsets_sum[2]+=ins_data.gyro[2];
      accel_offsets_sum[0]+=ins_data.accel[0];
      accel_offsets_sum[1]+=ins_data.accel[1];
      accel_offsets_sum[2]+=ins_data.accel[2];
      accel_offsets_sum[2]-=9.79;
      if(cnt==3000)
      {
         ins_data.gyro_offset[0]=gyro_offsets_sum[0]/cnt;
         ins_data.gyro_offset[1]=gyro_offsets_sum[1]/cnt;
         ins_data.gyro_offset[2]=gyro_offsets_sum[2]/cnt;
         ins_data.accel_offset[0]=accel_offsets_sum[0]/cnt;
         ins_data.accel_offset[1]=accel_offsets_sum[1]/cnt;
         ins_data.accel_offset[2]=accel_offsets_sum[2]/cnt;   
      }
      cnt++;
    }
  }
   
  void ist8310_init()
  {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
     osDelay(50);
     HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
     osDelay(50);
   // ist8310_IIC_write_single_reg(0x0B,0x08);   //�??????启中断，并设置成低电�??????
   //  HAL_Delay(150);
    ist8310_IIC_write_single_reg(0x41,0x09);   //平均采样两次
     osDelay(150);
     ist8310_IIC_write_single_reg(0x42,0xC0);  //必须�??????0xC0
     osDelay(150);
    ist8310_IIC_write_single_reg(0x0A,0x0B);  //200Hz输出频率
     osDelay(150);
  }


     void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
  {
      HAL_I2C_Mem_Write(&hi2c3, 0x0E <<1, reg,1,&data,1,10);

  }

     void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
  {
     HAL_I2C_Mem_Read(&hi2c3, 0x0E <<1, reg,I2C_MEMADD_SIZE_8BIT,buf,len,10);
  }

     void ist8310_read_mag(fp32 *mag)
{
     uint8_t buf[6];
     int16_t temp_ist8310_data = 0;
     //read the "DATAXL" register (0x03)
     ist8310_IIC_read_muli_reg(0x03, buf, 6);
     temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
     mag[0] = 0.3f * temp_ist8310_data;
     temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
     mag[1] = 0.3f * temp_ist8310_data;
     temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
     mag[2] = 0.3f * temp_ist8310_data;
}

 void AHRS_init(ins_data_t * ins_data)
 {
        ins_data->INS_quat[0] = 1.0f;
        ins_data->INS_quat[1] = 0.0f;
        ins_data->INS_quat[2] = 0.0f;
        ins_data->INS_quat[3] = 0.0f;
 }

     float invSqrt(float x)
 {
        float halfx = 0.5f * x;
	      float y = x;
	      long i = *(long*)&y;
	      i = 0x5f3759df - (i>>1);
	      y = *(float*)&i;
	      y = y * (1.5f - (halfx * y * y));
	      return y;
}

void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f)*57.3;
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]))*57.3;
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f)*57.3;
}
