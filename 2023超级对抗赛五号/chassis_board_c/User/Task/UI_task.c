#include "UI_task.h"
#include "CRC.h"
#include "judge.h"
uint8_t UI_Seq;                         //�����

uint8_t referee_uart_tx_buf[2][150];    //����ϵͳѧ�����ڷ���DMA�����
uint8_t referee_tx_fifo = 0;            //����ʹ�õĻ����


extern JUDGE_MODULE_DATA Judge_Hero;
extern UART_HandleTypeDef huart6;
static uint8_t usart6_tx_dma_is_busy = 0;


#define UI_NEED_REFRESH (userUI_control.module_refresh_timer > USERUI_MODULE_REFRESH_TIME)
#define UI_IS_EXTERN    (userUI_control.module_extern_flag)


/**
  * @brief          �Զ���UI��ʼ�������ݳ�ʼ�������Ʊ���Ԫ��
  * @param[in]      none
  * @retval         none
  */
void userUI_init(void);

/**
  * @brief          �Զ���UI���Ʊ���Ԫ��
  * @param[in]      none
  * @retval         none
  */
void userUI_draw_background(void);

/**
  * @brief          �Զ���UI������λ��׼��
  * @param[in]      ˮƽ��������������
  * @param[in]      ��ֱ�����������¸���
  * @param[in]      ���ű���
  * @retval         none
  */
void userUI_draw_foresight(int16_t delta_x, int16_t delta_y, fp32 scale);


//��Ҫ��ʾ��ͼ��
Graph_Data graph1, graph2, graph3, graph4, graph5, graph6, graph7;
//��Ҫ��ʾ���ַ���
String_Data ui_str;

userUI_control_t userUI_control;

/**
  * @brief          �Զ���UI������client�ϻ����Զ���UI
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void UI_Task(void const* argument)
{

     osDelay(USERUI_TASK_CONTROL_TIME_MS);
  
    //��ʼ��
    userUI_init();
    
    while (1)
    {
        userUI_init();
        osDelay(USERUI_TASK_CONTROL_TIME_MS);
    }
}


/**
  * @brief          �Զ���UI��ʼ�������ݳ�ʼ�������Ʊ���Ԫ��
  * @param[in]      none
  * @retval         none
  */
void userUI_init(void)
{
    //userUI_control.rc = get_remote_control_point();
    userUI_control.module_refresh_timer = 0;
    userUI_control.module_extern_flag = 0;

    //������λ�У�ͼ��1��
    userUI_draw_foresight(0, 0, 1.0f);
}




/**
  * @brief          �Զ���UI������λ��׼�У�����ͼ��1�ϣ�
  * @param[in]      ˮƽ�����������󸺣�
  * @param[in]      ��ֱ�����������¸���
  * @param[in]      ���ű�����������
  * @retval         none
  */
void userUI_draw_foresight(int16_t delta_x, int16_t delta_y, fp32 scale)
{
    Line_Draw(&graph1, "FS1", UI_Graph_ADD, 1, UI_Color_Cyan, 1, 960 + delta_x, 370 + delta_y, 960 + delta_x, 580 + delta_y);
    Line_Draw(&graph2, "FS2", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 900 + delta_x, 540 + delta_y, 1020 + delta_x, 540 + delta_y);
    Line_Draw(&graph3, "FS3", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 920 + delta_x, 510 + delta_y, 1000 + delta_x, 510 + delta_y);
    Line_Draw(&graph4, "FS4", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 930 + delta_x, 470 + delta_y, 990 + delta_x, 470 + delta_y);
    Line_Draw(&graph5, "FS5", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 940 + delta_x, 420 + delta_y, 980 + delta_x, 420 + delta_y);
    Line_Draw(&graph6, "FS6", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 950 + delta_x, 380 + delta_y, 970 + delta_x, 380 + delta_y);
    Circle_Draw(&graph7, "FS7", UI_Graph_ADD, 1, UI_Color_Cyan, 1, 960 + delta_x, 540 + delta_y, 20);
    ui_display_7_graph(&graph1, &graph2, &graph3, &graph4, &graph5, &graph6, &graph7);
}










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
void Line_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->operate_tpye = Graph_Operate;
    image->graphic_tpye = UI_Graph_Line;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}


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
 */
void Rectangle_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Rectangle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}


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
void Circle_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Circle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->radius = Graph_Radius;
}


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
void Arc_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t x_Length, uint32_t y_Length)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Arc;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_StartAngle;
    image->end_angle = Graph_EndAngle;
    image->end_x = x_Length;
    image->end_y = y_Length;
}



/************************************************���Ƹ���������*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Graph_Size     �ֺ�
        Graph_Digit    С��λ��
        Start_x��Start_x    ��ʼ����
        Graph_Float   Ҫ��ʾ�ı���
**********************************************************************************************************/

void Float_Draw(Float_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, float Graph_Float)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Float;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_Size;
    image->end_angle = Graph_Digit;
    image->graph_Float = Graph_Float;
}


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
void string_Draw(String_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Start_x, uint32_t Start_y, char* Char_Data)
{
    uint8_t i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->Graph_Control.graphic_name[2 - i] = imagename[i];
    image->Graph_Control.graphic_tpye = UI_Graph_Char;
    image->Graph_Control.operate_tpye = Graph_Operate;
    image->Graph_Control.layer = Graph_Layer;
    image->Graph_Control.color = Graph_Color;
    image->Graph_Control.start_x = Start_x;
    image->Graph_Control.start_y = Start_y;
    image->Graph_Control.width = Graph_Size;                //�߿�
    image->Graph_Control.start_angle = Graph_Digit;         //�ֺ�

    i = 0;
    while (Char_Data[i] != '\0' && i < 30)
    {
        image->show_Data[i] = Char_Data[i];
        i++;
    }
    image->Graph_Control.end_angle = i;                     //��¼�ַ�����
    while (i < 30)
    {
        image->show_Data[i++] = 0;
    }
    
}


/**
 *  @brief          ɾ��ͼ��
 *  @param[in]      ��������  0���ղ���   1��ɾ��ָ��ͼ��   2��ɾ������ͼ��
 *  @param[in]      ͼ���� [0, 9]
 */
void UI_delete(uint8_t operate_type, uint8_t layer)
{
    UI_Packhead head;
    UI_Data_Operate operate;
    UI_Data_Delete delete_layer;

    //1. ֡ͷ���ݳ�ʼ��
    head.SOF = 0xA5;
    head.Data_Length = sizeof(UI_Data_Operate) + sizeof(UI_Data_Delete);
    head.Seq = UI_Seq++;
    head.CMD_ID = 0x301;

    //2. �����˼�ͨѶ��ͷ��ʼ��
    operate.Data_ID = 0x100;                //�Զ���ID��ɾ��ͼ��
    operate.Sender_ID = get_robot_id();     //���Ͷ�ID
    operate.Receiver_ID = get_client_id();  //���ն�ID

    //3. ɾ������ת¼
    delete_layer.Delete_Operate = operate_type;
    delete_layer.Layer = layer;

    //4. ����ת��
    memcpy(referee_uart_tx_buf[referee_tx_fifo], &head, sizeof(UI_Packhead));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead), &operate, sizeof(UI_Data_Operate));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate), &delete_layer, sizeof(UI_Data_Delete));

    //5. CRCУ��
    Append_CRC8_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], 5);
    Append_CRC16_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(UI_Data_Delete) + 2);

    //6. DMA����
    while (get_usart6_tx_dma_busy_flag())
    {
        osDelay(1);
    }
    usart6_tx_dma_enable(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(UI_Data_Delete) + 2);
    referee_tx_fifo = referee_tx_fifo == 0 ? 1 : 0;
}


/**
 *  @brief          ʹ���Զ���UI����һ��ͼ�Σ�ˢ�³�������Ļ����ʾ
 *  @param[in]      ͼ��
 *  @return         none
 */
void ui_display_1_graph(Graph_Data* graph1)
{
    UI_Packhead head;
    UI_Data_Operate operate;

    //֡ͷ���ݳ�ʼ��
    head.SOF = 0xA5;
    head.Data_Length = sizeof(UI_Data_Operate) + sizeof(Graph_Data);  //Ӧ��Ϊ6+15
    head.Seq = UI_Seq++;
    head.CMD_ID = 0x301;

    //�����˼�ͨѶ��ͷ��ʼ��
    operate.Data_ID = 0x101;                //�Զ���ID
    operate.Sender_ID = get_robot_id();     //���Ͷ�ID
    operate.Receiver_ID = get_client_id();  //���ն�ID

    //����ת��
    memcpy(referee_uart_tx_buf[referee_tx_fifo], &head, sizeof(UI_Packhead));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead), &operate, sizeof(UI_Data_Operate));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate), graph1, sizeof(Graph_Data));

    //CRCУ��
    Append_CRC8_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], 5);
    Append_CRC16_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) + 2);

    //DMA����
    while (get_usart6_tx_dma_busy_flag())
    {
        osDelay(1);
    }
    usart6_tx_dma_enable(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) + 2);
    referee_tx_fifo = referee_tx_fifo == 0 ? 1 : 0;
}


/**
 *  @brief          ʹ���Զ���UI��������ͼ�Σ�ˢ�³�������Ļ����ʾ
 *  @param[in]      ͼ��1
 *  @param[in]      ͼ��2
 *  @return         none
 */
void ui_display_2_graph(Graph_Data* graph1, Graph_Data* graph2)
{
    UI_Packhead head;
    UI_Data_Operate operate;

    //֡ͷ���ݳ�ʼ��
    head.SOF = 0xA5;
    head.Data_Length = sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 2;        //Ӧ��Ϊ6+30
    head.Seq = UI_Seq++;
    head.CMD_ID = 0x301;

    //�����˼�ͨѶ��ͷ��ʼ��
    operate.Data_ID = 0x102;                //�Զ���ID
    operate.Sender_ID = get_robot_id();     //���Ͷ�ID
    operate.Receiver_ID = get_client_id();  //���ն�ID

    //����ת��
    memcpy(referee_uart_tx_buf[referee_tx_fifo], &head, sizeof(UI_Packhead));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead), &operate, sizeof(UI_Data_Operate));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate), graph1, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data), graph2, sizeof(Graph_Data));

    //CRCУ��
    Append_CRC8_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], 5);
    Append_CRC16_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 2 + 2);

    //DMA����
    while (get_usart6_tx_dma_busy_flag())
    {
        osDelay(1);
    }
    usart6_tx_dma_enable(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 2 + 2);
    referee_tx_fifo = referee_tx_fifo == 0 ? 1 : 0;
}


/**
 *  @brief          ʹ���Զ���UI�������ͼ�Σ�ˢ�³�������Ļ����ʾ
 *  @param[in]      ͼ��1
 *  @param[in]      ͼ��2
 *  @param[in]      ͼ��3
 *  @param[in]      ͼ��4
 *  @param[in]      ͼ��5
 *  @return         none
 */
void ui_display_5_graph(Graph_Data* graph1, Graph_Data* graph2, Graph_Data* graph3, Graph_Data* graph4, Graph_Data* graph5)
{
    UI_Packhead head;
    UI_Data_Operate operate;

    //1. ֡ͷ���ݳ�ʼ��
    head.SOF = 0xA5;
    head.Data_Length = sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 5;        //Ӧ��Ϊ6+75
    head.Seq = UI_Seq++;
    head.CMD_ID = 0x301;

    //2. �����˼�ͨѶ��ͷ��ʼ��
    operate.Data_ID = 0x103;                //�Զ���ID����ͻ��˷������ͼ��
    operate.Sender_ID = get_robot_id();     //���Ͷ�ID
    operate.Receiver_ID = get_client_id();  //���ն�ID

    //3, ����ת��
    memcpy(referee_uart_tx_buf[referee_tx_fifo], &head, sizeof(UI_Packhead));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead), &operate, sizeof(UI_Data_Operate));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate), graph1, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data), graph2, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 2, graph3, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 3, graph4, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 4, graph5, sizeof(Graph_Data));

    //4. CRCУ��
    Append_CRC8_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], 5);
    Append_CRC16_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 5 + 2);

    //5. DMA����
    while (get_usart6_tx_dma_busy_flag())
    {
        osDelay(1);
    }
    usart6_tx_dma_enable(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 5 + 2);
    referee_tx_fifo = referee_tx_fifo == 0 ? 1 : 0;
}


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
void ui_display_7_graph(Graph_Data* graph1, Graph_Data* graph2, Graph_Data* graph3, Graph_Data* graph4, Graph_Data* graph5, Graph_Data* graph6, Graph_Data* graph7)
{
    UI_Packhead head;
    UI_Data_Operate operate;

    //1. ֡ͷ���ݳ�ʼ��
    head.SOF = 0xA5;
    head.Data_Length = sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 7;        //Ӧ��Ϊ6+105
    head.Seq = UI_Seq++;
    head.CMD_ID = 0x301;

    //2. �����˼�ͨѶ��ͷ��ʼ��
    operate.Data_ID = 0x104;                //�Զ���ID����ͻ��˷����߸�ͼ��
    operate.Sender_ID = get_robot_id();     //���Ͷ�ID
    operate.Receiver_ID = get_client_id();  //���ն�ID

    //3, ����ת��
    memcpy(referee_uart_tx_buf[referee_tx_fifo], &head, sizeof(UI_Packhead));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead), &operate, sizeof(UI_Data_Operate));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate), graph1, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data), graph2, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 2, graph3, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 3, graph4, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 4, graph5, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 5, graph6, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 6, graph7, sizeof(Graph_Data));

    //4. CRCУ��
    Append_CRC8_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], 5);
    Append_CRC16_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 7 + 2);
  
    //5. DMA����
    while (get_usart6_tx_dma_busy_flag())
    {
        osDelay(1);
    }
    usart6_tx_dma_enable(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 7 + 2);
    referee_tx_fifo = referee_tx_fifo == 0 ? 1 : 0;
}


/**
 *  @brief          ʹ���Զ���UI�����ַ���
 *  @param[in]      �ַ������ݽṹ��ָ��
 *  @return         none
 */
void ui_display_string(String_Data* str_data)
{
    UI_Packhead head;
    UI_Data_Operate operate;

    //1. ֡ͷ���ݳ�ʼ��
    head.SOF = 0xA5;
    head.Data_Length = sizeof(UI_Data_Operate) + sizeof(String_Data);  //Ӧ��Ϊ6+45
    head.Seq = UI_Seq++;
    head.CMD_ID = 0x301;

    //2. �����˼�ͨѶ��ͷ��ʼ��
    operate.Data_ID = 0x110;                //�Զ���ID,�����ַ���
    operate.Sender_ID = get_robot_id();     //���Ͷ�ID
    operate.Receiver_ID = get_client_id();  //���ն�ID

    //3. ����ת��
    memcpy(referee_uart_tx_buf[referee_tx_fifo], &head, sizeof(UI_Packhead));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead), &operate, sizeof(UI_Data_Operate));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate), str_data, sizeof(String_Data));

    //4. CRCУ��
    Append_CRC8_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], 5);
    Append_CRC16_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(String_Data) + 2);

    //5. DMA����
    while (get_usart6_tx_dma_busy_flag())
    {
        osDelay(1);
    }
    usart6_tx_dma_enable(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(String_Data) + 2);
    referee_tx_fifo = referee_tx_fifo == 0 ? 1 : 0;
}







uint8_t get_usart6_tx_dma_busy_flag(void)
{
    return usart6_tx_dma_is_busy;
    //return hdma_usart6_tx.State;
}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    usart6_tx_dma_is_busy = 1;

    HAL_UART_Transmit_DMA(&huart6, data, len);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart6)
    {
        clear_usart6_tx_dma_busy_sign();
    }
}


void clear_usart6_tx_dma_busy_sign(void)
{
    usart6_tx_dma_is_busy = 0;
}



uint8_t get_robot_id(void)
{
    return Judge_Hero.robot_status.robot_id;
}

uint16_t get_client_id(void)
{
    if (Judge_Hero.robot_status.robot_id < 100)
    {
        return 0x100 + Judge_Hero.robot_status.robot_id;
    }
    else
    {
        return 0x164 + (Judge_Hero.robot_status.robot_id - 100);
    }
}




