#include "jy901s.h"

char imu_buffer[100];
char jytempdata[44];
uint8_t imurxflag;
uint8_t imurxsize;
/**
  * @brief  获取陀螺仪原始数据
  * @param  jy901s的数据结构体指针
  * @retval 无
  */
void JY901S_GetData(jydata* initdata)
{
    uint8_t k = 0;//传入指针数组索引
    uint8_t n = 0;//data的索引
        
    if(imurxflag == 1)
    {
        //一个包为11个数据
        if(imurxsize < 11)
        {
          imurxflag = 0;
          return;
        }
        else
        {
            //找到帧头0x55
            while(jytempdata[n] != 0x55|| jytempdata[n+11] != 0x55)
            {
                 n++;
                if(imurxsize - n < 10)
                {
                    imurxflag = 0;
                    return;
                }
            }
            for(uint8_t j = 0; j<3; j++)
            {
                switch(jytempdata[n+1+j*11])
                {
                    //根据第二个字节判断数据类型
                    case 0x51:
                    {
                        k = 0;
                        //只取3-8位
                        for(uint8_t i = n+2; i<7; i+=2)
                            initdata->acc[k++] = (jytempdata[i+1+j*11]<<8)|jytempdata[i+j*11];
                        break;
                    }
                    case 0x52:
                    {
                        k = 0;
                        for(uint8_t i = n+2; i<7; i+=2)
                            initdata->gyro[k++] = (jytempdata[i+1+j*11]<<8)|jytempdata[i+j*11];
                        break;
                    }
                    case 0x53:
                    {
                        k = 0;
                        for(uint8_t i = n+2; i<7; i+=2)
                            initdata->angle[k++] = (jytempdata[i+1+j*11]<<8)|jytempdata[i+j*11];
                        break;
                    }
                }
                
            }
            imurxflag = 0;
        }
    }
}

/**
  * @brief  获取陀螺仪单位转换后的数据
  * @param  jy901s的数据结构体指针
  * @retval 无
  */
jydata initdata;
void JY901S_DataConverse(imudata* data)
{
    JY901S_GetData(&initdata);
//    data->roll = (double)initdata.angle[0]*0.0054932;
//    data->pitch = (double)initdata.angle[1]*0.0054932;
//    data->yaw = (double)initdata.angle[2]*0.0054932;
    data->ax = (double)initdata.acc[0]*0.0004883;
    data->ay = (double)initdata.acc[1]*0.0004883;
    data->az = (double)initdata.acc[2]*0.0004883;
    data->gx = (double)initdata.gyro[0]*0.061035;
    data->gy = (double)initdata.gyro[1]*0.061035;
    data->gz = (double)initdata.gyro[2]*0.061035;
    data->yaw += data->gz*sampletime; 
    //控制在+-180
    data->yaw>180  ?data->yaw-=360 : 0;
    data->yaw<-180 ?data->yaw+=360 : 0;
    
    
    
}

