#include "jy901s.h"

/**
  * @brief  获取九轴数据和欧拉角
  * @param  jy901s数据结构体指针
  * @retval 无
  */

void JY901S_GetData(jydata* data)
{
    data->ax = I2C_ReadReg(JY901SADDRESSS, AX);
    data->ay = I2C_ReadReg(JY901SADDRESSS, AY); 
    data->az = I2C_ReadReg(JY901SADDRESSS, AZ);
    data->gz = I2C_ReadReg(JY901SADDRESSS, GZ);
    data->gz = I2C_ReadReg(JY901SADDRESSS, GZ);
    data->gz = I2C_ReadReg(JY901SADDRESSS, GZ);
    data->mz = I2C_ReadReg(JY901SADDRESSS, HZ);
    data->mz = I2C_ReadReg(JY901SADDRESSS, HZ);
    data->mz = I2C_ReadReg(JY901SADDRESSS, HZ);
    data->roll = I2C_ReadReg(JY901SADDRESSS, Roll);
    data->pitch = I2C_ReadReg(JY901SADDRESSS, Pitch);
    data->yaw = I2C_ReadReg(JY901SADDRESSS, Yaw);
}


//void JY901s_Process(void)
//{
//		if(JY901s.ReceiveNum < IMU_RXBUFFER_LEN) return;   	//如果位数不对

//		for(uint8_t i=0;i<9;i++)
//		{
//				if(JY901s.BuffTemp[i*11]!= JY901s.frame_head) return;	//如果帧头不对
//				switch(JY901s.BuffTemp[i*11+1])
//				{
//						case 0x51:
//							memcpy(&JY901s.acc.a,&JY901s.BuffTemp[2 + i*11],8);
//							for(uint8_t j = 0; j < 3; j++)
//						JY901s.acc.a[j] = JY901s.acc.a[j]/32768*16;
//						break;

//						case 0x52:
//							memcpy(&JY901s.w.w,&JY901s.BuffTemp[2 + i*11],8);
//							for(uint8_t j = 0; j < 3; j++)
//						JY901s.w.w[j] = JY901s.w.w[j]/32768*2000;
//						break;

//						case 0x53:
//							memcpy(&JY901s.angle.angle,&JY901s.BuffTemp[2 + i*11],8);
//							for(uint8_t j = 0; j < 3; j++)
//						JY901s.angle.angle[j] = JY901s.angle.angle[j]/32768*180;
//						break;

//						// case 0x54:	//磁场解算
//						// 	memcpy(&stcMag,&JY901s.BuffTemp[2 + i*11],8);
//						// 	for(uint8_t j = 0; j < 3; j++)
//						// JY901s.h.h[j] = (float)stcMag.h[j];
//						// break;
//						//
//						// case 0x55:	//D0-D3端口状态
//						// break;
//						//
//						// case 0x56:	//气压高度
//						// 	memcpy(&stcPress,&JY901s.BuffTemp[2 + i*11],8);
//						// JY901s.lPressure.lPressure = (float)stcPress.lPressure;
//						// JY901s.lPressure.lAltitude = (float)stcPress.lAltitude/100;
//						// break;
//						//
//						// case 0x57:	//经纬度
//						// 	memcpy(&stcLonLat.lLat,&JY901s.BuffTemp[2 + i*11],8);
//						// JY901s.lLon.lLat = (float)stcLonLat.lLat/10000000+(double)(stcLonLat.lLat % 10000000)/1e5;
//						// JY901s.lLon.lLat = (float)stcLonLat.lLon/10000000+(double)(stcLonLat.lLon % 10000000)/1e5;
//						// break;
//						//
//						// case 0x58:	//GPS
//						// break;
//						//
//						// case 0x59:	//四元数
//						// 	memcpy(&stcQ,&JY901s.BuffTemp[2 + i*11],8);
//						// 	for(uint8_t j = 0; j < 4; j++)
//						// JY901s.q.q[j] = (float)stcQ.q[j]/32768;
//						// break;
//				}
//		}
//}



