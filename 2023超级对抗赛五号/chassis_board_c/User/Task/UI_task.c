#include "UI_task.h"
extern uint8_t update_float_flag;
uint32_t i1=0;
uint8_t seq=0;
void UI_Task(void const *pvParameters)   
{
  
     osDelay(100);
    while(1)
    {
       
        if(i1%10==0)
        {
             Judge_Char_Send_to_Cilent();
        }
        if(i1%4==0)
        {
            update_float_flag = MODIFY;
            Client_supercap_remain_update(update_float_flag);
        }
       

      if(i1%100==0)
      {
        update_float_flag = ADD;
        Client_supercap_remain_update(update_float_flag);
      }
			if(seq==0xff)seq=0;
			else seq++;
    
        i1++;
      osDelay(50);
    }
 }
