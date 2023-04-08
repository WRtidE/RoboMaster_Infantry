
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "remote_control.h"
#include "math.h"
#include "PID.h"
#include "insTask.h"

int flag;


uint32__t shoot_flag;
pid_struct_t pitch_pid[7];	
fp32 target_angle;




osThreadId defaultTaskHandle;
osThreadId weopenHandle;
osThreadId can_6020_pitchHandle;
osThreadId communicationHandle;
osThreadId insHandle;



void StartDefaultTask(void const * argument);
void weopen_task(void const * argument);
void gimbal_pitch_task(void const * argument);
void communication_task(void const * argument);
void insTask(void const * argument);

void MX_FREERTOS_Init(void);

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void MX_FREERTOS_Init(void) {






  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(weopen, weopen_task, osPriorityIdle, 0, 128);
  weopenHandle = osThreadCreate(osThread(weopen), NULL);

  osThreadDef(can_6020_pitch, gimbal_pitch_task, osPriorityIdle, 0, 128);
  can_6020_pitchHandle = osThreadCreate(osThread(can_6020_pitch), NULL);

  osThreadDef(communication, communication_task, osPriorityIdle, 0, 128);
  communicationHandle = osThreadCreate(osThread(communication), NULL);

  osThreadDef(ins, insTask, osPriorityIdle, 0, 128);
  insHandle = osThreadCreate(osThread(ins), NULL);


}

void StartDefaultTask(void const * argument)
{
  //HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
  for(;;)
  {
    osDelay(1);
  }
}

void weopen_task(void const * argument)
{

  
  pid_init(&motor_pid[0], 5, 0.01, 0, 30000, 30000); 
  pid_init(&motor_pid[1], 5, 0.01, 0, 30000, 30000); 
  pid_init(&motor_pid[4], 10, 0.1, 0, 30000, 30000); 
  pid_init(&pitch_pid[3], 40, 3, 20, 30000, 30000); 
  for(;;)
  {


    if( (rc_ctrl.rc.s[1] == 2 && shoot_flag == 0)||shoot_flag == 2)
    {
      target_angle = motor_info[3].rotor_angle;
    }

    if (rc_ctrl.rc.s[1] == 2)
    {
      target_speed[0] = -5000;
      target_speed[1] = 5000;
      target_speed[4] = 400;
      target_speed[3] = 400;
      motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
      motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
      motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
      motor_info[3].set_voltage = pid_calc(&pitch_pid[3], target_angle, motor_info[3].rotor_angle);
      shoot_flag = 1;
    }else
    {  
      target_speed[0] = 0;
      target_speed[1] = 0;
      target_speed[4] = 0;
      motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
      motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
      motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
      shoot_flag = 0;
    }

    set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage, motor_info[2].set_voltage,motor_info[3].set_voltage);
    set_motor_voltage(1, motor_info[4].set_voltage, 0,0,0);
    osDelay(1);
  }
}

void gimbal_pitch_task(void const * argument)
{
  pid_init(&motor_pid[3], 80, 3, 0, 30000, 30000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 30000  
  for(;;)
  {
    if(can_flag ==1)
    {
      if (rc_ctrl.rc.ch[1] >= 974 && rc_ctrl.rc.ch[1] <= 1074) 
      {
        if (motor_info[3].rotor_speed > 10 || motor_info[3].rotor_speed < -10) //锟斤拷锟斤拷(?)
        {
          target_speed[3] = 0;
          motor_info[3].set_voltage = pid_calc(&motor_pid[3], target_speed[3], motor_info[3].rotor_speed);
        }
        else
        {
          target_speed[3] = 0;
        }
      }else
      {
        if(rc_ctrl.rc.ch[1]>=364&&rc_ctrl.rc.ch[1]<=824 &&motor_info[3].rotor_angle < 7800) //遥控器往下拨
        {
          target_speed[3] = -((rc_ctrl.rc.ch[1] - 1024) / 660 * 16);
          motor_info[3].set_voltage = pid_calc(&motor_pid[3], target_speed[3], motor_info[3].rotor_speed);
          shoot_flag = 2;
        }
        if(rc_ctrl.rc.ch[1]>=1224&&rc_ctrl.rc.ch[1]<=1684&& motor_info[3].rotor_angle>5800) //遥控器往上拨
        {
          target_speed[3] = -((rc_ctrl.rc.ch[1] - 1024) / 660 * 16);
          motor_info[3].set_voltage = pid_calc(&motor_pid[3], target_speed[3], motor_info[3].rotor_speed);
          shoot_flag = 2;
        }
      }
      set_motor_voltage1(0,motor_info[1].set_voltage, motor_info[2].set_voltage, motor_info[3].set_voltage, motor_info[3].set_voltage);
    }
    osDelay(1);
  }
}

void communication_task(void const * argument)
{
  // motor_info[1].set_voltage = 0;
  // motor_info[2].set_voltage = 0;
  // motor_info[3].set_voltage = 0;
  // motor_info[4].set_voltage = 0;
  // motor_info[5].set_voltage = 0;
  // motor_info[6].set_voltage = 0;
  // motor_info[7].set_voltage = 0;
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



