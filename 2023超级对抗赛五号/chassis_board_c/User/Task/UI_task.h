#ifndef __RM_CILENT_UI__
#define __RM_CILENT_UI__
#include "main.h"
#include "stm32f4xx.h"
#include "stdarg.h"
#include "struct_typedef.h"
#include "rc_potocal.h"
#include "drv_can.h"

#define Robot_ID UI_Data_RobotID_RStandard1
#define Cilent_ID UI_Data_CilentID_RStandard1        //�첽��1

#pragma pack(1)                           //��1�ֽڶ���

#define NULL 0
#define __FALSE 100

/****************************��ʼ��־*********************/
#define UI_SOF 0xA5
/****************************CMD_ID����********************/
#define UI_CMD_Robo_Exchange 0x0301    
/****************************����ID���ݣ��ⲿ��Ϊ�Զ���********************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101				//��һ��ͼ
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104				//��7��ͼ
#define UI_Data_ID_DrawChar 0x110			//д�ַ�
/****************************�췽������ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************����������ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************�췽������ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************����������ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************ɾ������***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************ͼ�����ò���__ͼ�β���********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************ͼ�����ò���__ͼ������********************/
#define UI_Graph_Line 0         //ֱ��
#define UI_Graph_Rectangle 1    //����
#define UI_Graph_Circle 2       //��Բ
#define UI_Graph_Ellipse 3      //��Բ
#define UI_Graph_Arc 4          //Բ��
#define UI_Graph_Float 5        //������
#define UI_Graph_Int 6          //����
#define UI_Graph_Char 7         //�ַ���
/***************************ͼ�����ò���__ͼ����ɫ********************/
#define UI_Color_Main 0         //������ɫ
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //�Ϻ�ɫ
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //��ɫ
#define UI_Color_Black 7
#define UI_Color_White 8



typedef unsigned char Uint8_t;
typedef unsigned char uint8_t;



//֡ͷ����
typedef __packed struct
{
    uint8_t SOF;                    //��ʼ�ֽ�,�̶�0xA5
    uint16_t Data_Length;           //֡���ݳ���
    uint8_t Seq;                    //�����
    uint8_t CRC8;                   //CRC8У��ֵ
    uint16_t CMD_ID;                //����ID
} UI_Packhead;

//���0x0301����� �Զ���ID
typedef __packed struct
{
    uint16_t Data_ID;               //����ID ���Զ���
    uint16_t Sender_ID;             //������ID
    uint16_t Receiver_ID;           //������ID
} UI_Data_Operate;

//�ͻ���ͼ�β���
typedef __packed struct
{
    uint8_t Delete_Operate;         //ɾ������
    uint8_t Layer;                  //ɾ��ͼ��
} UI_Data_Delete;          //ɾ��ͼ��֡


typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    float graph_Float;              //��������
} Float_Data;


typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11;              //ͼ������
} Graph_Data;


typedef __packed struct
{
    Graph_Data Graph_Control;
    uint8_t show_Data[30];
} String_Data;                  //��ӡ�ַ�������




/**
 *  @brief          ɾ��ͼ��
 *  @param[in]      ��������  0���ղ���   1��ɾ��ָ��ͼ��   2��ɾ������ͼ��
 *  @param[in]      ͼ���� [0, 9]
 */
 void UI_delete(uint8_t operate, uint8_t layer);

/**
 *  @brief          ʹ���Զ���UI����ֱ��
 *  @param[out]     Graph_Data*
 *  @param[in]      ͼ������ �����ַ�
 *  @param[in]      ͼ�β���
 *  @param[in]      ͼ�� 0-9
 *  @param[in]      ͼ����ɫ
 *  @param[in]      ͼ���߿�
 *  @param[in]      ��ʼX����
 *  @param[in]      ��ʼY����
 *  @param[in]      ����X����
 *  @param[in]      ����Y����
 *  @retval         none
 */
void Line_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

/**
 *  @brief          ʹ���Զ���UI������Բ
 *  @author         ɽ������ѧ
 *  @param[out]     Graph_Data*
 *  @param[in]      ͼ������ �����ַ�
 *  @param[in]      ͼ�β���
 *  @param[in]      ͼ�� 0-9
 *  @param[in]      ͼ����ɫ
 *  @param[in]      ͼ���߿�
 *  @param[in]      Բ��X����
 *  @param[in]      Բ��Y����
 *  @param[in]      �뾶
 *  @retval         none
 */
 void Circle_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);

/**
 *  @brief          ʹ���Զ���UI����Բ��
 *  @author         ɽ������ѧ
 *  @param[out]     Graph_Data*
 *  @param[in]      ͼ������ �����ַ�
 *  @param[in]      ͼ�β���
 *  @param[in]      ͼ�� 0-9
 *  @param[in]      ͼ����ɫ
 *  @param[in]      ͼ���߿�
 *  @param[in]      ��ʼ�Ƕ�
 *  @param[in]      ��ֹ�Ƕ�
 *  @param[in]      Բ��X����
 *  @param[in]      Բ��Y����
 *  @param[in]      X�����᳤
 *  @param[in]      Y�����᳤���ο���Բ��ʹ�÷���
 *  @retval         none
 */
 void Arc_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t x_Length, uint32_t y_Length);

/**
 *  @brief          ʹ���Զ���UI���ƾ���
 *  @author         ɽ������ѧ
 *  @param[out]     Graph_Data*
 *  @param[in]      ͼ������ �����ַ�
 *  @param[in]      ͼ�β���
 *  @param[in]      ͼ�� 0-9
 *  @param[in]      ͼ����ɫ
 *  @param[in]      ͼ���߿�
 *  @param[in]      ��ʼX����
 *  @param[in]      ��ʼY����
 *  @param[in]      ����X����
 *  @param[in]      ����Y����
 *  @return         none
 */
 void Rectangle_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);

void Float_Draw(Float_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, float Graph_Float);

/**
 *  @brief          �����ַ�������
 *  @param[in]      String_Data* �ַ����ṹ���ָ��
 *  @param[in]      ͼƬ����
 *  @param[in]      ͼ�β���
 *  @param[in]      ͼ�� [0,9]
 *  @param[in]      ͼ����ɫ
 *  @param[in]      ͼ���߿�
 *  @param[in]      �ֺţ��������߿�ı�Ϊ 10:1
 *  @param[in]      ��ʼ����X
 *  @param[in]      ��ʼ����Y
 *  @param[in]      �����͵��ַ�����һ����෢��30���ַ�
 *  @retval         none
 */
 void string_Draw(String_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Start_x, uint32_t Start_y, char* Char_Data);
/**
 *  @brief          ʹ���Զ���UI����һ��ͼ�Σ�ˢ�³�������Ļ����ʾ
 *  @param[in]      ͼ��
 *  @return         none
 */
 void ui_display_1_graph(Graph_Data* graph1);

/**
 *  @brief          ʹ���Զ���UI��������ͼ�Σ�ˢ�³�������Ļ����ʾ
 *  @param[in]      ͼ��1
 *  @param[in]      ͼ��2
 *  @return         none
 */
 void ui_display_2_graph(Graph_Data* graph1, Graph_Data* graph2);

/**
 *  @brief          ʹ���Զ���UI�������ͼ�Σ�ˢ�³�������Ļ����ʾ
 *  @param[in]      ͼ��1
 *  @param[in]      ͼ��2
 *  @param[in]      ͼ��3
 *  @param[in]      ͼ��4
 *  @param[in]      ͼ��5
 *  @return         none
 */
 void ui_display_5_graph(Graph_Data* graph1, Graph_Data* graph2, Graph_Data* graph3, Graph_Data* graph4, Graph_Data* graph5);

/**
 *  @brief          ʹ���Զ���UI�����߸�ͼ�Σ�ˢ�³�������Ļ����ʾ
 *  @param[in]      ͼ��1
 *  @param[in]      ͼ��2
 *  @param[in]      ͼ��3
 *  @param[in]      ͼ��4
 *  @param[in]      ͼ��5
 *  @param[in]      ͼ��6
 *  @param[in]      ͼ��7
 *  @return         none
 */
 void ui_display_7_graph(Graph_Data* graph1, Graph_Data* graph2, Graph_Data* graph3, Graph_Data* graph4, Graph_Data* graph5, Graph_Data* graph6, Graph_Data* graph7);

/**
 *  @brief          ʹ���Զ���UI�����ַ���
 *  @param[in]      �ַ������ݽṹ��ָ��
 *  @return         none
 */
 void ui_display_string(String_Data* str_data);



#define USERUI_TASK_CONTROL_TIME_MS 10

//�����ֶ�ˢ�¼�һ��ʱ�䣬UIȫ���ػ棨��UI�����쳣ʱʹ�ã�
#define USERUI_MANUAL_REFRESH_CONTROL_TIME  1000
#define USERUI_MANUAL_REFRESH_KEY           KEY_PRESSED_OFFSET_Z

//ÿ��һ��ʱ�䣬����UI�ؼ����������������ػ�һ��
#define USERUI_MODULE_REFRESH_TIME          1000






typedef struct  
{
    const RC_ctrl_t* rc;
    uint16_t all_refresh_key_press_time;    //ȫ��ˢ�°������µ�ʱ��
    uint8_t all_refresh_flag;               //ȫ��ˢ�±�־λ
    uint16_t module_refresh_timer;          //ÿ��һ��ʱ�䣬����UI�ؼ����������������ػ�һ��
    uint8_t module_extern_flag;             //UIԪ���Ѵ��ڱ�־
} userUI_control_t;

void UI_Task(void const* argument);
uint8_t get_usart6_tx_dma_busy_flag(void);
void usart6_tx_dma_enable(uint8_t *data, uint16_t len);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
void clear_usart6_tx_dma_busy_sign(void);

uint8_t get_robot_id(void);
uint16_t get_client_id(void);

#endif