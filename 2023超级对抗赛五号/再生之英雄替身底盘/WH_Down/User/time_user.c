#include "time_user.h"
void TIM1_UP_TIM10_IRQHandler(void)//��ʱ��1���жϣ�һ��CubeMx��������ϵͳ�ļ����棬������Ҫɾ��ϵͳ�ļ��е��Ǹ�
{
	if(motor_info[4].rotor_angle)//���ݽǶ��������ж��Ƿ���յ��˵������Ӧ
	{
		flag_shoot=1;//��1��ʼת��
		time_count++;//���õ�TIM1��100Hz��Ҳ����0.01s��һ���ж�
	}
	if(time_count==time)//������������ʱ�䣬Ŀǰ�ǲ�ȡ��1s�����븳��һ��time��ֵ����Ȼ��ʱ����ʼ����δ��ֱֵ�ӻ�ֹͣ��ʱ��(bug)
	{
		flag_shoot=0;//��0ֹͣת��
		time_count=0;//��������0
		HAL_TIM_Base_Stop_IT(&htim1);//ֹͣ��ʱ��
	}
  HAL_TIM_IRQHandler(&htim1);//�����ϵͳ������������б�־λ��0
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//�жϻص�������Ϊɶû����
{

  UNUSED(htim);


}