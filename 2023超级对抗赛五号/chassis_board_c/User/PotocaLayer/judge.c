#include "judge.h"
#include "CRC.h"
#include "main.h"
#include "struct_typedef.h"
#include "drv_can.h"
JUDGE_MODULE_DATA Judge_Robot;

extern UART_HandleTypeDef huart6;

robot_data_t infantry;


void update_data();//定义一些需要用到的变量并实时更新数值方便其他文件调用
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
                        memcpy((void*)(&Judge_Robot.status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0003:
                        data_length = 32;
                         memcpy((void*)(&Judge_Robot.robot_hp), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0005:
                        data_length = 13;
                          memcpy((void*)(&Judge_Robot.zone), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0101:
                        data_length = 4;
                        memcpy((void*)(&Judge_Robot.event_data), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0102:
                        data_length = 4;
                         memcpy((void*)(&Judge_Robot.supply_status), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0104 :
                        data_length = 2;
                        memcpy((void*)(&Judge_Robot.warning), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0105 :
                        data_length = 1;
                        memcpy((void*)(&Judge_Robot.dart_remaining_time), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0201:
                         data_length = 27;
                         memcpy((void*)(&Judge_Robot.robot_status), (const void*)(&databuffer[pos+7]), data_length);   //底盘功率限制上限在这
                        break;
                    case 0x0202:
                        data_length = 16;
                        memcpy((void*)(&Judge_Robot.power_heat), (const void*)(&databuffer[pos+7]), data_length);      //含实时功率热量数据
                        break;
                    case 0x0203:
                        data_length = 16;
                         memcpy((void*)(&Judge_Robot.robot_pos), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0204:
                        data_length = 1;
                        memcpy((void*)(&Judge_Robot.buff), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0205:
                        data_length = 1;
                        memcpy((void*)(&Judge_Robot.aerial_energy), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0206:
                        data_length =1;
                        memcpy((void*)(&Judge_Robot.robot_hurt), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0207:
                        data_length = 7;
                        memcpy((void*)(&Judge_Robot.shoot_data), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0208:
                        data_length = 6;
                        memcpy((void*)(&Judge_Robot.bullet_remain), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0209:
                        data_length = 4;
                        memcpy((void*)(&Judge_Robot.rfid_status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x020A:
                        data_length = 6;
                        memcpy((void*)(&Judge_Robot.rfid_status), (const void*)(&databuffer[pos+7]), data_length);
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
	update_data();
	judge_data_send(infantry.shooter_heat, infantry.shooter_heat_limit ,infantry.Robot_level,0);
	
}


void update_data()
{
	infantry.Robot_id = Judge_Robot.robot_status.robot_id;//ID号
	infantry.Robot_level = Judge_Robot.robot_status.robot_level;//等级
	infantry.chassis_power = Judge_Robot.power_heat.chassis_power;   //底盘功率
	infantry.chassis_power_buffer = Judge_Robot.power_heat.chassis_power_buffer; //底盘缓冲功率
	infantry.total_current = Judge_Robot.power_heat.chassis_current; //底盘电流
	infantry.shooter_heat = Judge_Robot.power_heat.shooter_id1_17mm_cooling_heat; //枪口热量
	infantry.shooter_heat_limit = Judge_Robot.robot_status.shooter_id1_17mm_cooling_limit; //枪口热量上限
}






