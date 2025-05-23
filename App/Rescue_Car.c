#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"
#include "jy901s.h"

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

void Data_Task(void* pv)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        
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
    while(1)
    {
       
       
        vTaskDelay(100);
    }
}


void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        if(__HAL_DMA_GET_COUNTER(&hdma_usart2_rx)==0)
            rxflag = 1;//打开接收标志，进数据处理函数
        HAL_UART_Receive_DMA(&huart2, (uint8_t*) rxbuffer, 44);
    }
}
