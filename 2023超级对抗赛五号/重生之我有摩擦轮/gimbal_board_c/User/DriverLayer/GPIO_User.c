#include "GPIO_User.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//GPIO���ⲿ�ж�
{
    if(GPIO_Pin == GPIO_PIN_3)
    {
        ist8310_read_mag(mag);//���ж�����ȡist8310������
    }

}