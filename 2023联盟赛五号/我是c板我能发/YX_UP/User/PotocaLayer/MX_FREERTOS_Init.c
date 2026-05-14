#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "MX_FREERTOS_Init.h"
#include "Yaw_task.h"
#include "StartDefaultTask.h"
#include "Pitch_task.h"
#include "Exchange_task.h"
#include "Friction_task.h"
#include "INS_task.h"

osThreadId insTaskHandle;
osThreadId fixTaskHandle;
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
void MX_FREERTOS_Init(void) {
  
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, Pitch_task, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */

  osThreadDef(myTask04, Friction_task, osPriorityIdle, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	osThreadDef(imutask, INS_Task,  osPriorityRealtime, 0, 512);	//�����ǽ�������
  insTaskHandle = osThreadCreate(osThread(imutask), NULL);
		

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}