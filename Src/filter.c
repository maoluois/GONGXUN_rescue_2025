#include "filter.h"
float mean_fliter(float *data, int n) {
    float sum = 0;
    for (int i = 0; i < n; i++) {
        sum += data[i];
    }
    return sum / n;
}


float Lowpass_fliter(float data, float last_data, float a) {
    return a * data + (1 - a) * last_data;
}

//定义各个传感器的数据采样点列表
float m_dataList[MAX_SENSOR_NUM][MAX_DATA_NUM] = {0};

//冒泡排序
void BubbleSort(float array[], int len)
{
    float temp;
    //外层循环控制排序的趟数，n个元素排序需要循环n-1次
    for(int i = 0; i < len - 1; i ++)
    {
        //内层循环控制比较的次数，n个元素第i趟比较n-i次
        for(int j = 0; j < len - 1 - i; j ++)
        {
            //比较相邻的元素大小 目的：将最大的元素选出到移动到最后
            if(array[j] > array[j+1])
            {
                temp = array[j];
                array[j] = array[j+1];
                array[j+1] = temp;
            }
        }
    }
}

//滑窗均值滤波，这里采样点data数据类型和滤波后返回值数据类型都是double，实际使用可根据需要定义
//其他数据类型
float Filter_SlidingWindowAvg(int index, float data)
{
    static int dataNum[MAX_SENSOR_NUM] = {0}; //定义记录传感器的采样点个数
    int i;
    float sum = 0;
    float out = 0;
    float array[MAX_DATA_NUM] = {0};

    //数据采样点在采样窗口内移动，FIFO操作
    for(i = MAX_DATA_NUM - 2; i >= 0; i--)
        m_dataList[index][i+1] = m_dataList[index][i];

    m_dataList[index][0] = data;
    //数据采样点数量小于采样窗口长度，对采样窗口数据累加后进行平均值运算
    if (dataNum[index] < MAX_DATA_NUM)
    {
        dataNum[index] ++;
        for(i = 0; i < dataNum[index]; i++)
        {
            sum += m_dataList[index][i];
        }
        out = sum / dataNum[index];
    }
    // 数据采样点已填满采样窗口，进行排序后，去除n个最大值及最小值后，对滤波窗口内的数据累加后进
    // 行平均值运算
    else
    {
        for(i = 0; i < MAX_DATA_NUM; i++)
        {
            array[i] = m_dataList[index][i];
        }
        //调用冒泡排序函数
        BubbleSort(array, MAX_DATA_NUM);

        int start = (MAX_DATA_NUM - WINDOW_DATA_NUM) / 2; //start = REMOVE_MAXMIN_NUM

        for(i = start; i < start + WINDOW_DATA_NUM; i++)
        {
            sum += array[i];
        }
        out = sum / WINDOW_DATA_NUM;
    }
    return out;
}
