#include "jy901s.h"

char rxbuffer[100];
uint8_t rxflag;

void JY901S_GetData(jydata* intialdata)
{
    char data[44];
    uint8_t k;//传入指针数组索引
    uint8_t i;//data的索引
    if(rxflag == 1)
    {
        rxflag = 0;
        memcpy(data, rxbuffer, 44);
        
        for(uint8_t j = 0; j<4; j++)
        {
            k = 0;
            if (data[j*11]!=0x55)//判断帧头
                break;

            switch(data[1+j*11])
            {
                //根据第二个字节判断数据类型
                case 0x51:
                {
                    //只取3-8位
                    for(i = 2; i<7; i+=2)
                        intialdata->acc[k++] = (data[i+j*11]<<8)|data[i+1+j*11];
                    break;
                }
                case 0x52:
                {
                    for(i = 2; i<7; i+=2)
                        intialdata->gyro[k++] = (data[i+j*11]<<8)|data[i+1+j*11];
                    break;
                }
                case 0x53:
                {
                    for(i = 2; i<7; i+=2)
                        intialdata->angle[k++] = (data[i+j*11]<<8)|data[i+1+j*11];
                    break;
                }
                case 0x54:
                {
                    for(i = 2; i<7; i+=2)
                        intialdata->mag[k++] = (data[i+j*11]<<8)|data[i+1+j*11];
                    break;
                }
            }
        }
    }
}


