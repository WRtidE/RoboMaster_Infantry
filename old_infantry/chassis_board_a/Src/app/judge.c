#include "judge.h"
#include "CRC.h"
#include "main.h"
#include "struct_typedef.h"
JUDGE_MODULE_DATA judge_Robot;






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
                CMD_ID = (databuffer[pos+1]&0xff)|((databuffer[pos+2]<<8)&0xff00);
                switch(CMD_ID)
                {
                    case 0x0001:
                        data_length = 11;
                        memcpy((void*)(&judge_Robot.status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0003:
                        data_length = 32;
                         memcpy((void*)(&judge_Robot.robot_hp), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0005:
                        data_length = 13;
                          memcpy((void*)(&judge_Robot.zone), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0101:
                        data_length = 4;
                        memcpy((void*)(&judge_Robot.event_data), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0102:
                        data_length = 4;
                         memcpy((void*)(&judge_Robot.supply_status), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0104 :
                        data_length = 2;
                        memcpy((void*)(&judge_Robot.warning), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0105 :
                        data_length = 1;
                        memcpy((void*)(&judge_Robot.dart_remaining_time), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0201:
                         data_length = 27;
                         memcpy((void*)(&judge_Robot.robot_status), (const void*)(&databuffer[pos+7]), data_length);   //底盘功率限制上限在这
                        break;
                    case 0x0202:
                        data_length = 16;
                        memcpy((void*)(&judge_Robot.power_heat), (const void*)(&databuffer[pos+7]), data_length);      //含实时功率热量数据
                        break;
                    case 0x0203:
                        data_length = 16;
                         memcpy((void*)(&judge_Robot.robot_pos), (const void*)(&databuffer[pos+7]), data_length);
                         break;
                    case 0x0204:
                        data_length = 1;
                        memcpy((void*)(&judge_Robot.buff), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0205:
                        data_length = 1;
                        memcpy((void*)(&judge_Robot.aerial_energy), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0206:
                        data_length =1;
                        memcpy((void*)(&judge_Robot.robot_hurt), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0207:
                        data_length = 7;
                        memcpy((void*)(&judge_Robot.shoot_data), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0208:
                        data_length = 6;
                        memcpy((void*)(&judge_Robot.bullet_remain), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x0209:
                        data_length = 4;
                        memcpy((void*)(&judge_Robot.rfid_status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    case 0x020A:
                        data_length = 6;
                        memcpy((void*)(&judge_Robot.rfid_status), (const void*)(&databuffer[pos+7]), data_length);
                        break;
                    default:break;
                }


                pos+=data_length+9;
                continue;

            }






          }








        }



        pos++;
     


     }



}
