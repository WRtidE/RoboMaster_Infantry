#ifndef STARTTASK04_H
#define STARTTASK04_H

#include "INS_task.h"
#include "pid.h"
#include "struct_typedef.h"
#include  "drv_can.h"

//����һЩ�����ļ���ȫ�ֱ���
extern ins_data_t ins_data;		//���������ǽ������������
extern RC_ctrl_t rc_ctrl;		//ң��������
extern uint16_t Up_ins_yaw;

//����һЩ�궨��


//���庯��
void StartTask04(void const * argument);


#endif
