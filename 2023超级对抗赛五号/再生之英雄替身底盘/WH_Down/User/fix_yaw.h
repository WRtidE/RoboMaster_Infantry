#ifndef FIX_YAW_H
#define FIX_YAW_H

#include "INS_task.h"
#include "pid.h"
#include "struct_typedef.h"
#include "remote_control.h"

//����һЩ�����ļ���ȫ�ֱ���
extern ins_data_t ins_data;		//���������ǽ������������
extern RC_ctrl_t rc_ctrl;		//ң��������


//����һЩ�궨��


//���庯��
void FIX_Task(void const *pvParameters);


#endif