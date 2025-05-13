#ifndef __JY901S_H
#define __JY901S_H

#include "MyI2C.h"
#include "jy901s_register.h"

typedef struct jydata
{
    int16_t ax, ay, az;
    int16_t gy, gx, gz;
    float mx, my, mz;
    float roll, pitch, yaw;
}jydata;

void JY901S_GetData(jydata* data);
    
//#define DIO_MODE_AIN 0
//#define DIO_MODE_DIN 1
//#define DIO_MODE_DOH 2
//#define DIO_MODE_DOL 3
//#define DIO_MODE_DOPWM 4
//#define DIO_MODE_GPS 5

//struct STime
//{
//	unsigned char ucYear;
//	unsigned char ucMonth;
//	unsigned char ucDay;
//	unsigned char ucHour;
//	unsigned char ucMinute;
//	unsigned char ucSecond;
//	unsigned short usMiliSecond;
//};

//struct SAcc//加速度
//{
//	short a[3];
//	short T;
//};
//struct SGyro//角速度
//{
//	short w[3];
//	short T;
//};
//struct SAngle//角度
//{
//	short Angle[3];
//	short T;
//};
//struct SMag//磁场输出
//{
//	short h[3];
//	short T;
//};

//struct SDStatus//端口状态数据输出
//{
//	short sDStatus[4];
//};

//struct SPress//气压高度
//{
//	long lPressure;
//	long lAltitude;
//};

//struct SLonLat//经纬度
//{
//	long lLon;
//	long lLat;
//};

//struct SGPSV
//{
//	short sGPSHeight;
//	short sGPSYaw;
//	long lGPSVelocity;
//};
//struct SQ //四元数
//{ short q[4];
//};

//// **********************************************************************************************************

//typedef struct
//{
//	float angle[3];
//}Angle;

//typedef struct
//{
//	float a[3];
//}Acc;

//typedef struct
//{
//	float w[3];
//}SGyro;


//typedef struct//四元数
//{ float q[4];
//}SQ;

//typedef struct//磁场输出
//{
//	float h[3];
//}SMag;

//typedef struct//气压高度
//{
//	float lPressure;
//	float lAltitude;
//}SPress;

//typedef struct//经纬度
//{
//	float lLon;
//	float lLat;
//}SLonLat;

//// 将数据和串口配置集成在一起
//typedef struct
//{
//	uint16_t ReceiveNum;
//	uint8_t frame_head;					//帧头
//	uint8_t BuffTemp[IMU_RXBUFFER_LEN];		//接收缓冲
//	Angle angle;						//角度
//	Acc acc;								//加速度
//	SGyro w;								//角速度
//	SMag H;									//磁场
//	SPress lPressure;   	  //气压
//	SPress lAltitude;     	//高度
//	SLonLat lLon;						//经度
//	SLonLat lLat;						//维度
//	SQ q; 									//四元数
//} JY_USART;

//extern JY_USART JY901s;

//void JY901s_Process(void);
//void JY901s_Init(JY_USART *Data);

#endif

