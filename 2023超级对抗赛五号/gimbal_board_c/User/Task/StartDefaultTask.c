 #include "StartDefaultTask.h"
 #include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
 #include "INS_task.h"
 
 

 void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}
