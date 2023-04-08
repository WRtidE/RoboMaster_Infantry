
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"


#include "remote_control.h"
#include "math.h"
#include "PID.h"

osThreadId defaultTaskHandle;
osThreadId communicationHandle;
osThreadId can_6020_yawHandle;
osThreadId can_3508Handle;






void StartDefaultTask(void const * argument);
void communication_task(void const * argument);
void gimbal_yaw_task(void const * argument);
void chassis_task(void const * argument);

void MX_FREERTOS_Init(void);


void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );


static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
 
}


/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
 

 
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

 
  osThreadDef(communication, communication_task, osPriorityIdle, 0, 128);
  communicationHandle = osThreadCreate(osThread(communication), NULL);

 
  osThreadDef(can_6020_yaw, gimbal_yaw_task, osPriorityIdle, 0, 128);
  can_6020_yawHandle = osThreadCreate(osThread(can_6020_yaw), NULL);

 
  osThreadDef(can_3508, chassis_task, osPriorityIdle, 0, 128);
  can_3508Handle = osThreadCreate(osThread(can_3508), NULL);

 

 
 

}


/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */

void StartDefaultTask(void const * argument)
{
 
 
  for(;;)
  {
    osDelay(1);
  }
 
}


/**
* @brief Function implementing the can1_com thread.
* @param argument: Not used
* @retval None
*/

void communication_task(void const * argument)
{
 
  
 
  motor_info[0].set_voltage = 0;
  motor_info[1].set_voltage = 0;
  motor_info[2].set_voltage = 0;
  motor_info[3].set_voltage = 0; 
  motor_info[4].set_voltage = 0;
  for (;;)
  {
    if (rc_ctrl.rc.ch[3] > 300)
    {
      can_flag = 1;
      HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET); 
    }
    osDelay(1);
  }
 
}


/**
* @brief Function implementing the can_6020_yaw thread.
* @param argument: Not used
* @retval None
*/

void gimbal_yaw_task(void const * argument)
{
 
 
  pid_init(&motor_pid[4], 40, 3, 0, 30000, 30000); 
  for(;;)
  {
    if(can_flag==1)
    {
      if (rc_ctrl.rc.ch[0] >= 974 && rc_ctrl.rc.ch[0] <= 1074) 
      {
        if (motor_info[4].rotor_speed > 10 || motor_info[4].rotor_speed < -10) 
        {
          target_speed[4] = 0;
          motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
        }
        else
        {
          motor_info[4].set_voltage = 0;
        }
      }else
      {
        if(rc_ctrl.rc.ch[0]>=364&&rc_ctrl.rc.ch[0]<=1684)
        {
          target_speed[4] = -((rc_ctrl.rc.ch[0] - 1024) / 660 * 20); 
          motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
        }
      }
      set_motor_voltage1(1, motor_info[4].set_voltage, motor_info[5].set_voltage, motor_info[6].set_voltage, 0);
    }
    osDelay(1);
  }
 
}


/**
* @brief Function implementing the can_3508_down thread.
* @param argument: Not used
* @retval None
*/

void chassis_task(void const * argument)
{
 
  
 
  for (uint8_t i = 0; i < 4; i++) 
  {
    pid_init(&motor_pid[i], 30, 0, 0, 16384, 16384); 
  }                                                   

  for (;;)
  {
    for (uint8_t i = 0; i < 4; i++)
    {                                                                                                  
      motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed); 
    }                                                                                                  
    if (can_flag == 1)                                                                                 
    {

      if ((rc_ctrl.rc.ch[2] >= 974 && rc_ctrl.rc.ch[2] <= 1074) && ((rc_ctrl.rc.ch[3] >= 974) && (rc_ctrl.rc.ch[3] <= 1074)) && (rc_ctrl.rc.ch[4] <= 1074) && (rc_ctrl.rc.ch[4] >= 974))
      {
        for (int i = 0; i < 4; i++) 
        {
          if (motor_info[i].rotor_speed > 720 || motor_info[i].rotor_speed < -720)
          {
            target_speed[i] = 0;
            motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed);
          }
          else
          {
            motor_info[i].set_voltage = 0;
          }
        }
      }
      else
      {
        if (rc_ctrl.rc.ch[4] > 1074)
        {
          target_curl = sqrt((rc_ctrl.rc.ch[4] - 1024) * (rc_ctrl.rc.ch[4] - 1024)) / 660;
        } 
        else if (rc_ctrl.rc.ch[4] < 974)
        {
          target_curl = -(sqrt((rc_ctrl.rc.ch[4] - 1024) * (rc_ctrl.rc.ch[4] - 1024)) / 660);
        }
        else
        {
          target_curl = 0;
        }
        target_curl = target_curl * 16384;
        int16_t target_curl_int = target_curl;

        r = sqrt((rc_ctrl.rc.ch[3] - 1024) * (rc_ctrl.rc.ch[3] - 1024) + (rc_ctrl.rc.ch[2] - 1024) * (rc_ctrl.rc.ch[2] - 1024));
        sin_sita = (1024 - rc_ctrl.rc.ch[3]) / r;
        cos_sita = (rc_ctrl.rc.ch[2] - 1024) / r;
        target_v = (r / 660) * 16384 ;

        if (target_curl == 0)
        {
          target_speed[0] = (0.707 * target_v * (sin_sita - cos_sita))/2;
          target_speed[1] = -(0.707 * target_v * (sin_sita + cos_sita))/2;
          target_speed[2] = -(0.707 * target_v * (sin_sita - cos_sita))/2;
          target_speed[3] = (0.707 * target_v * (sin_sita + cos_sita))/2; 
        }
        else
        {
          target_int1 = (0.707 * target_v * (sin_sita - cos_sita)) / 2; 
          target_int2 = (0.707 * target_v * (sin_sita + cos_sita)) / 2;

          target_speed[0] = (target_int1 - target_curl_int) / 2;
          target_speed[1] = (-target_int2 - target_curl_int) / 2;
          target_speed[2] = (-target_int1 - target_curl_int) / 2;
          target_speed[3] = (target_int2 - target_curl_int) / 2; 
        }
      }
    }
    else 
    {
      motor_info[0].set_voltage = 0;
      motor_info[1].set_voltage = 0;
      motor_info[2].set_voltage = 0;
      motor_info[3].set_voltage = 0;
    }
    
    set_motor_voltage(0,
                      motor_info[0].set_voltage,
                      motor_info[1].set_voltage,
                      motor_info[2].set_voltage,
                      motor_info[3].set_voltage);
    osDelay(1);
  }
 
}





