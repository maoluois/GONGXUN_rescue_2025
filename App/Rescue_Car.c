#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"
#include "jy901s.h"
#include "Xbox.h"
#include "Algorithm.h"

#define RESCUE_CAR_START_STACK                 128
#define RESCUE_CAR_START_PRIORITY              1
TaskHandle_t rescue_car_start_handle;
void Rescue_Car_Start(void* pv);

#define DATA_TASK_STACK                 128
#define DATA_TASK_STACK_PRIORITY        4
TaskHandle_t data_task_handle;
void Data_Task(void* pv);

#define CONTROL_TASK_STACK              128
#define CONTROL_TASK_STACK_PRIORITY     3
TaskHandle_t contro_task_handle;
void Control_Task(void* pv);

#define SHOW_TASK_STACK                 128
#define SHOW_TASK_STACK_PRIORITY        2
TaskHandle_t show_task_handle;
void Show_Task(void* pv);

void Rescue_Car_Init(void)
{
    xTaskCreate( (TaskFunction_t) Rescue_Car_Start,
                (char *) "Rescue_Car_Start", 
                (configSTACK_DEPTH_TYPE) RESCUE_CAR_START_STACK,
                (void *) NULL,
                (UBaseType_t) RESCUE_CAR_START_PRIORITY,
                (TaskHandle_t *) &rescue_car_start_handle );
    vTaskStartScheduler();
}

void Rescue_Car_Start(void* pv)
{
    taskENTER_CRITICAL();
    
    xTaskCreate( (TaskFunction_t) Data_Task,
                (char *) "Data_Task", 
                (configSTACK_DEPTH_TYPE) DATA_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) DATA_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &data_task_handle );
                
    xTaskCreate( (TaskFunction_t) Control_Task,
                (char *) "Control_Task", 
                (configSTACK_DEPTH_TYPE) CONTROL_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) CONTROL_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &contro_task_handle );
                
    xTaskCreate( (TaskFunction_t) Show_Task,
                (char *) "Show_Task", 
                (configSTACK_DEPTH_TYPE) SHOW_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) SHOW_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &show_task_handle );
                
    vTaskDelete(NULL);
                
    taskEXIT_CRITICAL();
}
/*===========================================
上面是任务初始化，下面是执行的任务
==============================================*/
imudata angle;
void Data_Task(void* pv)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        JY901S_DataConverse(&angle);
        printf("%.1lf\r\n", angle.yaw);
        xTaskDelayUntil(&pxPreviousWakeTime, 10);
    }
}

void Control_Task(void* pv)
{
    
    
    while(1)
    {
        vTaskDelay(100);
    }
}

void Show_Task(void* pv)
{
    uint8_t xb_data[4] = {0,0,0,0};
    while(1)
    {
       if(xUART8.ReceiveNum != 0)
       {
           xUART8.ReceiveNum = 0;
           Get_Data_Xbox(xb_data);
            if (XboxData[0] != 0 && XboxData[0] != 1 && XboxData[1] != 0 && XboxData[1] != 1)
            {
//                calculate_target_speeds(XboxData[2], XboxData[3], &SpeedY, &angular_speed);
                //死区防止静止时抖动
//                if (SpeedY < 3.99 && SpeedY > -3.99)
//              {
//                  SpeedY = 0;
//                  // printf("%f\n", SpeedY);
//              }
//              if (angular_speed < 0.05 && angular_speed > -0.05)
//              {
//                  angular_speed = 0;
//                  // printf("%f\n", angular_speed);
//              }

            }
       }
        vTaskDelay(50);
    }
}


/*===================================================================================*/
//各种回调
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
    //IMU--USART2串口回调
    if(huart->Instance == USART2)
    {
        if(__HAL_DMA_GET_COUNTER(&hdma_usart2_rx)==0)
        {
            imurxflag = 1;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
   
    if(huart->Instance == USART2)
    {
        
        if(imurxflag == 0)
        {
            memcpy(jytempdata, imu_buffer, Size);
            imurxflag = 1;
            imurxsize = Size;
        }
            HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*) imu_buffer, 44);
    }
    
    if (huart == &huart8)                                                                    // 判断串口
    {
        __HAL_UNLOCK(huart);                                                                 // 解锁串口状态
        xUART8.ReceiveNum  = Size;                                                          // 把接收字节数，存入结构体xUSART8.ReceiveNum，以备使用
        memset(xUART8.ReceiveData, 0, sizeof(xUART8.ReceiveData));                         // 清0前一帧的接收数据
        memcpy(xUART8.ReceiveData, xUART8.BuffTemp, Size);                                 // 把新数据，从临时缓存中，复制到xUSART8.ReceiveData[], 以备使用
        HAL_UARTEx_ReceiveToIdle_DMA(&huart8, xUART8.BuffTemp, sizeof(xUART8.BuffTemp));   // 再次开启DMA空闲中断; 每当接收完指定长度，或者产生空闲中断时，就会来到这个
// 其实，在CubeMX配置中，DMA有一个选项 ：Mode的circular, 可以让DMA进行连续地的工作，接收完成后，无需在回调函数里再次开启DMA 。但是，目前的CubeMX版本(V6.10），这个参数的选择，会使我们上面的DMA接收与发送，相冲突。那我们二选一好了，自行手工调用。
    }
}
