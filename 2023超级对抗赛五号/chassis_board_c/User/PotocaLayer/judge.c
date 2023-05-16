#include "judge.h"
#include "CRC.h"
#include "main.h"
#include "struct_typedef.h"
extern UART_HandleTypeDef huart6;
JUDGE_MODULE_DATA Judge_Hero;
ext_power_heat_data_t powerd;
float newpower;
extern uint8_t seq;
uint8_t Hero_level;
uint8_t Hero_id;
uint16_t Hero_42mm_speed_limit;
uint16_t Hero_chassis_power_limit;
fp32 Hero_42mm_speed;

extern float powerdata[4];
#define LEN_CMD_ID 2
#define LEN_FRAME_HEAD 5
uint8_t first_line[30] = {"spin:"};
uint8_t CliendTxBuffer[200];


void Update_data();//定义一些需要用到的变量并实时更新数值方便其他文件调用
void JUDGE_Receive(volatile uint8_t *databuffer,uint8_t length)
{
    uint8_t pos=0;
    uint16_t data_length=0;
    uint16_t CMD_ID =0;
    
     while(pos<length)
     {
        if(databuffer[pos]==0xA5)
        {
            if(Verify_CRC8_Check_Sum(&databuffer[pos],5))
            {
                data_length = (databuffer[pos+1]&0xff)|((databuffer[pos+2]<<8)&0xff00);
                if(pos+data_length+9>length)
                {
                    continue;
                }
            if(Verify_CRC16_Check_Sum(&databuffer[pos],data_length+9))
            {
              
             
                CMD_ID = (databuffer[pos+5]&0xff)|((databuffer[pos+6]<<8)&0xff00);
                switch(CMD_ID)
                { 
                    case 0x0001:
                        data_length = 11;
                        memcpy((void*)(&Judge_Hero.status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0003:
                        data_length = 32;
                         memcpy((void*)(&Judge_Hero.robot_hp), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0005:
                        data_length = 13;
                          memcpy((void*)(&Judge_Hero.zone), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0101:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.event_data), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0102:
                        data_length = 4;
                         memcpy((void*)(&Judge_Hero.supply_status), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0104 :
                        data_length = 2;
                        memcpy((void*)(&Judge_Hero.warning), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0105 :
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.dart_remaining_time), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0201:
                         data_length = 27;
                         memcpy((void*)(&Judge_Hero.robot_status), (const void*)(&databuffer[pos+7]), data_length);                    //底盘功率限制上限在这
                         Determine_ID();
                        break;
                    case 0x0202:
                        data_length = 16;
                        memcpy((void*)(&Judge_Hero.power_heat), (const void*)(&databuffer[pos+7]), data_length);        //含实时功率热量数据
                        break;
                    case 0x0203:
                        data_length = 16;
                         memcpy((void*)(&Judge_Hero.robot_pos), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0204:
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.buff), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0205:
                        data_length = 1;
                        memcpy((void*)(&Judge_Hero.aerial_energy), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0206:
                        data_length =1;
                        memcpy((void*)(&Judge_Hero.robot_hurt), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0207:
                        data_length = 7;
                        memcpy((void*)(&Judge_Hero.shoot_data), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0208:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.bullet_remain), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0209:
                        data_length = 4;
                        memcpy((void*)(&Judge_Hero.rfid_status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x020A:
                        data_length = 6;
                        memcpy((void*)(&Judge_Hero.rfid_status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    default:break;
                }
                pos+=(data_length+9);
                continue;

            }
          }
        }
        pos++;
     
     }
		 Update_data();
}


void Update_data()
{
	Hero_id = Judge_Hero.robot_status.robot_id;//ID号
	Hero_level = Judge_Hero.robot_status.robot_level;//等级
	Hero_42mm_speed_limit = Judge_Hero.robot_status.shooter_id1_42mm_speed_limit;
	Hero_chassis_power_limit = Judge_Hero.robot_status.chassis_power_limit;
	newpower = powerd.chassis_power;
	if(Judge_Hero.shoot_data.bullet_speed)
	{
		Hero_42mm_speed = Judge_Hero.shoot_data.bullet_speed;
	}
}


void Determine_ID(void)//判断自己是哪个队伍
{		
	if(Judge_Hero.robot_status.robot_id < 10)//本机器人的ID，红方
	{ 
		Judge_Hero.ids.teammate_hero 		 	= 1;
		Judge_Hero.ids.teammate_engineer  = 2;
		Judge_Hero.ids.teammate_infantry3 = 3;
		Judge_Hero.ids.teammate_infantry4 = 4;
		Judge_Hero.ids.teammate_infantry5 = 5;
		Judge_Hero.ids.teammate_plane		 	= 6;
		Judge_Hero.ids.teammate_sentry		= 7;
		
		Judge_Hero.ids.client_hero 		 	= 0x0101;
		Judge_Hero.ids.client_engineer  = 0x0102;
		Judge_Hero.ids.client_infantry3 = 0x0103;
		Judge_Hero.ids.client_infantry4 = 0x0104;
		Judge_Hero.ids.client_infantry5 = 0x0105;
		Judge_Hero.ids.client_plane			= 0x0106;
		
		if     (Judge_Hero.robot_status.robot_id == hero_red)//不断刷新放置在比赛中更改颜色
			Judge_Hero.self_client = Judge_Hero.ids.client_hero;
		else if(Judge_Hero.robot_status.robot_id == engineer_red)
			Judge_Hero.self_client = Judge_Hero.ids.client_engineer;
		else if(Judge_Hero.robot_status.robot_id == infantry3_red)
			Judge_Hero.self_client = Judge_Hero.ids.client_infantry3;
		else if(Judge_Hero.robot_status.robot_id == infantry4_red)
			Judge_Hero.self_client = Judge_Hero.ids.client_infantry4;
		else if(Judge_Hero.robot_status.robot_id == infantry5_red)
			Judge_Hero.self_client = Judge_Hero.ids.client_infantry5;
		else if(Judge_Hero.robot_status.robot_id == plane_red)
			Judge_Hero.self_client = Judge_Hero.ids.client_plane;
	}
	else //蓝方
	{
		Judge_Hero.ids.teammate_hero 		 	= 101;
		Judge_Hero.ids.teammate_engineer  = 102;
		Judge_Hero.ids.teammate_infantry3 = 103;
		Judge_Hero.ids.teammate_infantry4 = 104;
		Judge_Hero.ids.teammate_infantry5 = 105;
		Judge_Hero.ids.teammate_plane		 	= 106;
		Judge_Hero.ids.teammate_sentry		= 107;
		
		Judge_Hero.ids.client_hero 		 	= 0x0166;
		Judge_Hero.ids.client_engineer  = 0x0167;
		Judge_Hero.ids.client_infantry3 = 0x0168;
		Judge_Hero.ids.client_infantry4 = 0x0169;
		Judge_Hero.ids.client_infantry5 = 0x0169;
		Judge_Hero.ids.client_plane			= 0x016A;
		
		if     (Judge_Hero.robot_status.robot_id == hero_blue)
			Judge_Hero.self_client = Judge_Hero.ids.client_hero;
		else if(Judge_Hero.robot_status.robot_id == engineer_blue)
			Judge_Hero.self_client = Judge_Hero.ids.client_engineer;
		else if(Judge_Hero.robot_status.robot_id == infantry3_blue)
			Judge_Hero.self_client = Judge_Hero.ids.client_infantry3;
		else if(Judge_Hero.robot_status.robot_id == infantry4_blue)
			Judge_Hero.self_client = Judge_Hero.ids.client_infantry4;
		else if(Judge_Hero.robot_status.robot_id == infantry5_blue)
			Judge_Hero.self_client = Judge_Hero.ids.client_infantry5;
		else if(Judge_Hero.robot_status.robot_id == plane_blue)
			Judge_Hero.self_client = Judge_Hero.ids.client_plane;
		
	}

}


/*----------------------------------------------------------------绘字符串*/
ext_charstring_data_t tx_client_char;


void Char_Graphic(ext_client_string_t* graphic,//最终要发出去的数组中的数据段内容
									const char* name,
									uint32_t operate_tpye,
									
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t length,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
                  
									const char *character)//外部放入的数组
{
	graphic_data_struct_t *data_struct = &graphic->grapic_data_struct;
	for(char i=0;i<3;i++)
		data_struct->graphic_name[i] = name[i];	//字符索引
	data_struct->operate_tpye = operate_tpye; //图层操作
	data_struct->graphic_tpye = CHAR;         //Char型
	data_struct->layer = layer;//都在第零层
	data_struct->color = color;//都是白色
	data_struct->start_angle = size;
	data_struct->end_angle = length;	
	data_struct->width = width;
	data_struct->start_x = start_x;
	data_struct->start_y = start_y;	
	
	data_struct->radius = 0;
	data_struct->end_x = 0;
	data_struct->end_y = 0;
	
//	memcpy(graphic->data,character,length);
}

void Judge_Char_Send_to_Cilent()
{
		//帧头
		tx_client_char.txFrameHeader.sof = 0xA5;
		tx_client_char.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(ext_client_string_t);
		tx_client_char.txFrameHeader.seq = seq;//包序号
		memcpy(CliendTxBuffer,&tx_client_char.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));
	
		//命令码
		tx_client_char.CmdID = 0x0301;
		
		//数据段头结构
		tx_client_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
		tx_client_char.dataFrameHeader.send_ID     = Judge_Hero.robot_status.robot_id;
		tx_client_char.dataFrameHeader.receiver_ID = Judge_Hero.self_client;
		
  
		Char_Graphic(&tx_client_char.clientData,"CL1",ADD,0,WHITE,10,strlen(first_line),1,(50),(1080*9/12),first_line);//x1920/18
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_char.CmdID, LEN_CMD_ID+tx_client_char.txFrameHeader.data_length);//加上命令码长度2
		
		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_client_char));
		//Client_Sent_String(CliendTxBuffer, sizeof(tx_client_char));
     HAL_UART_Transmit_DMA(&huart6,CliendTxBuffer, sizeof(tx_client_char));
}


/*----------------绘象形--------*/
ext_graphic_five_data_t tx_client_graphic_figure;
void Figure_Graphic(graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
									uint32_t layer,
									uint32_t color,
									uint32_t start_angle,
									uint32_t end_angle,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									uint32_t radius,
									uint32_t end_x,
									uint32_t end_y)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//字符索引
	graphic->operate_tpye = operate_tpye; //图层操作
	graphic->graphic_tpye = graphic_tpye;         //Char型
	graphic->layer        = layer;//都在第一层
	graphic->color        = color;//变色
	graphic->start_angle  = start_angle;
	graphic->end_angle    = end_angle;	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->radius = radius;
	graphic->end_x  = end_x;
	graphic->end_y  = end_y;
}



/*----------------绘浮点--------*/

ext_float_one_data_t tx_supercap_remain;
uint8_t update_float_flag = ADD;
void Float_Graphic(Float_data_struct_t* graphic,//最终要发出去的数组的数据段内容
									const char* name,
									uint32_t operate_tpye,
									uint32_t graphic_tpye,//绘制什么图像
									uint32_t layer,
									uint32_t color,
									uint32_t size,
									uint32_t decimal,
									uint32_t width,
									uint32_t start_x,
									uint32_t start_y,
									int number)							
{
	for(char i=0;i<3;i++)
		graphic->graphic_name[i] = name[i];	//字符索引
	graphic->operate_tpye = operate_tpye; //图层操作
	graphic->graphic_tpye = graphic_tpye;  
	graphic->layer        = layer;//
	graphic->color        = color;//变色
	graphic->start_angle  = size;
	graphic->end_angle    = decimal;//小数有效位	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->number       = number;
}

void Client_supercap_remain_update(uint8_t flag)
{
		//帧头
		tx_supercap_remain.txFrameHeader.sof = 0xA5;
		tx_supercap_remain.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(graphic_data_struct_t);
		tx_supercap_remain.txFrameHeader.seq = seq;//包序号
		memcpy(CliendTxBuffer,&tx_supercap_remain.txFrameHeader,sizeof(std_frame_header_t));
		Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

		//命令码
		tx_supercap_remain.CmdID = 0x0301;

		//数据段头结构
		tx_supercap_remain.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_one_graphic;
		tx_supercap_remain.dataFrameHeader.send_ID     = Judge_Hero.robot_status.robot_id;
		tx_supercap_remain.dataFrameHeader.receiver_ID = Judge_Hero.self_client;
	
		//数据段
		Float_Graphic(&tx_supercap_remain.clientData,"SC1",flag,FLOAT,1,YELLOW,30,2,3,(1920*4/6),810,(int)(powerdata[1]*1000))	;
		memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_supercap_remain.CmdID, LEN_CMD_ID+tx_supercap_remain.txFrameHeader.data_length);//加上命令码长度2

		//帧尾
		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_supercap_remain));
		
		//Client_Sent_String(CliendTxBuffer, sizeof(tx_supercap_remain));
    HAL_UART_Transmit_DMA(&huart6,CliendTxBuffer, sizeof(tx_supercap_remain));
}



void Usart6_Sent_Byte(uint8_t ch)
{
	USART6->DR = ch;
	while((USART6->SR&0x40)==0);
}

void Usart6_Sent_string(uint8_t *string, uint16_t length)
{
	uint16_t i;
	for(i = 0; i<length; i++)
	{
		Usart6_Sent_Byte(string[i]);
	}
}

void Client_Sent_String(uint8_t *string, uint16_t length)
{
  	Usart6_Sent_string(string, length);
}