#include "GPIO_User.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//GPIO的外部中断
{
    if(GPIO_Pin == GPIO_PIN_3)
    {
        ist8310_read_mag(mag);//靠中断来读取ist8310的数据
    }

}