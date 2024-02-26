#include "./454software/rtos_level.h"
//-----------------------------------------------------------------------
//                       底层驱动信息
//-----------------------------------------------------------------------
extern volatile MotorStatus motor_status; // 电机
extern volatile SensorData sensor_data;   // ADC(包含电磁阀状态,氧气压力,氧气浓度)
extern int16_t temperature_P7;            // P7_get();温度
extern int16_t temperature_P9;            // P9_get();温度
extern int16_t pressure_P10;              // P10_get();压力
extern int16_t pressure_P11;              // P11_get();压力
extern int16_t pressure_P12;              // P12_get();
extern uint16_t flow_P13;                 // P13_get();流量
extern uint16_t flow_P14;                 // P14_get();流量
extern uint16_t flow_P15;                 // P15_get();流量
// LED控制
//   LED1(ON),LED1(OFF)
//   LED2(ON),LED2(OFF)
//   LED3(ON),LED3(OFF)
//  电磁阀控制
//   P4_PWM_set(0);-->参数0-100
//   P5_PWM_set(0);-->参数0-100
//   P6_PWM_set(0);-->参数0-100
// 压电阀片控制
//   YDP_control(ON),YDP_control(OFF)
// 雾化接口器控制
//   Pulverizer_control(ON),Pulverizer_control(OFF),

//-----------------------------------------------------------------------
//                       rtos_level
//-----------------------------------------------------------------------
//! 余辉教授文档需求
extern QueueHandle_t USART0_RX_xQueue;
extern volatile struRtSample RtSample;
extern SemaphoreHandle_t RtSample_Mutex;
extern SemaphoreHandle_t iicCollect_Ready_Semaphore;
extern SemaphoreHandle_t iicCollect_Start_Semaphore;

//-----------------------------------------------------------------------
//                       任务函数原型
//-----------------------------------------------------------------------
void Test_report(void *pvParameters);
void led_toggle(void *pvParameters);
void iicTask(void *pvParameters);
void gTaskRtSample(void *pvParameters);
void gTaskRtMotor(void *pvParameters);
void gTaskCommMonitor(void *pvParameters);
void gTaskLocalRecSave(void *pvParameters);
//-----------------------------------------------------------------------
//                       主函数
//-----------------------------------------------------------------------
int main()
{
    init_454();
    // SD_init_454();
    printf("initinitialize finished!\n");
    // 获取校准数据
    // align_data();

    // 初始化互斥量和信号量
    RtSample_Mutex = xSemaphoreCreateMutex();
    iicCollect_Ready_Semaphore = xSemaphoreCreateBinary();
    iicCollect_Start_Semaphore = xSemaphoreCreateBinary();
    USART0_RX_xQueue = xQueueCreate(256, sizeof(char));

    motor_control(0);
    delay_ms_454(100);

    //! Heap_Size       EQU     0x00019000---->100KB
    // 创建3个 IIC 采集任务

    // xTaskCreate(iicTask, "IIC1_Task", 1024, (void *)1, 50, NULL);
    // xTaskCreate(iicTask, "IIC0_Task", 1024, (void *)2, 50, NULL);
    // xTaskCreate(iicTask, "IIC2_Task", 1024, (void *)3, 50, NULL);

    // xTaskCreate(gTaskRtSample, "gTaskRtSample", 1024, (void *)1, 40, NULL);
    // xTaskCreate(gTaskRtMotor, "gTaskRtMotor", 1024, (void *)1, 40, NULL);
    // xTaskCreate(gTaskCommMonitor, "gTaskCommMonitor", 1024, (void *)1, 50, NULL);
    // xTaskCreate(gTaskLocalRecSave, "gTaskLocalRecSave", 1024, (void *)1, 10, NULL);

    // xTaskCreate(Test_report, "Test_report", 1024, (void *)1, 40, NULL);
    xTaskCreate(led_toggle, "led_toggle", 1024, (void *)1, 40, NULL);
    // 启动调度器
    vTaskStartScheduler();
    for (;;)
        ;
    return 0;
}
//-----------------------------------------------------------------------
//                       任务实现
//-----------------------------------------------------------------------
void led_toggle(void *pvParameters)
{
    RtcTimeConfig(24, 2, 22, 4, 14, 28, 00);
    while (1)
    {
        gpio_bit_toggle(GPIOG, GPIO_PIN_6); // RED LED
        vTaskDelay(1000);
        rtc_show_time();
    }
}
void Test_report(void *pvParameters)
{
    // printf("Test report on board\n");
    //  motor_pressure_flow(0, 50);

    // printf("Test report with RTOS\n");
    //  RTOS_motor_pressure_flow(0,50);

    // printf("DEBUG hardware\n");
    // temperature_P7 = P7_get();
    // temperature_P9 = P9_get();
    // pressure_P10 = P10_get();
    // pressure_P11 = P11_get();
    // pressure_P12 = P12_get();
    // flow_P13 = P13_get();
    // flow_P14 = P14_get();
    // flow_P15 = P15_get();
    while (1)
    {
        gpio_bit_toggle(GPIOG, GPIO_PIN_7); // GREEN LED
        vTaskDelay(1000);
    }
}
void iicTask(void *pvParameters)
{
    int taskId = (int)pvParameters;

    while (1)
    {
        // 进入等待信号量状态
        xSemaphoreTake(iicCollect_Start_Semaphore, portMAX_DELAY);
        switch (taskId)
        {
        case 1:
            // IIC1
            flow_P13 = P13_get();
            pressure_P12 = P12_get();
            break;
        case 2:
            // IIC0
            flow_P14 = P14_get();
            pressure_P10 = P10_get();
            break;
        case 3:
            // IIC2
            flow_P15 = P15_get();
            pressure_P11 = P11_get();
            break;
        default:
            break;
        }
        // 发送信号量，通知定时任务可以读取数据
        xSemaphoreGive(iicCollect_Ready_Semaphore);
    }
}

void gTaskRtSample(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / SAMPLE_RATE); // 根据 SAMPLE_RATE 计算延时

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        xSemaphoreGive(iicCollect_Start_Semaphore);
        xSemaphoreTake(iicCollect_Ready_Semaphore, portMAX_DELAY);
        // 获取互斥量
        if (xSemaphoreTake(RtSample_Mutex, (TickType_t)50) == pdTRUE)
        {
            // 采集数据并更新数据结构
            RtSample.uIndex = xLastWakeTime; // 直接使用当前的TickCount
            RtSample.s16T1 = P7_get();
            RtSample.s16T1 = P9_get();
            RtSample.s16T1 = motor_status.motor_temperature;
            RtSample.s16P1 = pressure_P10;
            RtSample.s16P1 = pressure_P11;
            RtSample.s16P1 = pressure_P12;
            RtSample.u16F1 = flow_P13;
            RtSample.u16F2 = flow_P14;
            RtSample.u16F3 = flow_P15;
            RtSample.u16MotorSV = motor_status.current_speed;
            RtSample.uO2 = sensor_data.oxygen;
            RtSample.uValve1 = sensor_data.p4_valve;
            RtSample.uValve2 = sensor_data.p5_valve;
            RtSample.uValve3 = sensor_data.p6_valve;
            // 释放互斥量
            xSemaphoreGive(RtSample_Mutex);
        }
        else
        {
            // 错误处理：互斥量获取失败
        }
    }
}
void gTaskRtMotor(void *pvParameters)
{
    // 格式化状态报文
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 10); // 10Hz, 即每100毫秒执行一次

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. 获取控制参数并控制电机
        // MotorControlParams motorControlParams = gAlgorithm.Model.CalcPID(&gData.Target);
        // motor_control(motorControlParams.speed);

        // 2. 更新全局数据结构
        // 在任务gTaskRtSample中更新

        // 3. 生成状态报文并放入发送队列
        upload_struRtSample();
    }
}
void gTaskCommMonitor(void *pvParameters)
{
    printf("processDataFrame");
    char rxByte;
    UBaseType_t queueMessagesWaiting = 0;
    static char USART0_RX_frameBuffer[256];
    static uint16_t USART0_RX_bufferIndex = 0;
    for (;;)
    {
        if (xQueueReceive(USART0_RX_xQueue, &rxByte, portMAX_DELAY))
        {
            // 解析数据帧
            USART0_RX_frameBuffer[USART0_RX_bufferIndex++] = rxByte;
            // 检查是否收到完整的帧头
            if (USART0_RX_bufferIndex >= 3)
            {
                if (USART0_RX_frameBuffer[0] == 0xEB && USART0_RX_frameBuffer[1] == 0x90 && USART0_RX_frameBuffer[2] == 0xAA)
                {
                    uint8_t dataLength = USART0_RX_frameBuffer[3];
                    // 检查是否收到了完整的数据帧
                    if (USART0_RX_bufferIndex >= (dataLength + 4))
                    {
                        // 处理数据帧
                        processDataFrame(USART0_RX_frameBuffer, USART0_RX_bufferIndex);
                        // 重置缓冲区索引，为接收新帧做准备
                        USART0_RX_bufferIndex = 0;
                    }
                }
                else
                {
                    // 如果帧头不匹配，重置缓冲区索引并从新位置开始填充
                    // 如果帧头不匹配，这意味着当前接收到的数据不是有效的帧起始，因此尝试通过移除第一个字节来查找下一个可能的帧头
                    memmove(USART0_RX_frameBuffer, USART0_RX_frameBuffer + 1, --USART0_RX_bufferIndex);
                }
            }
            // 检查缓冲区是否已满
            if (USART0_RX_bufferIndex == 256)
            {
                // 缓冲区溢出，需要进行错误处理
                USART0_RX_bufferIndex = 0; // 重置缓冲区索引
            }
        }
    }
}

void gTaskLocalRecSave(void *pvParameters)
{
    // 待完成
}
//-----------------------------------------------------------------------
//                       HOOK
//-----------------------------------------------------------------------
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // 打印发生堆栈溢出的任务名称
    printf("\nStack overflow in task: ");
    printf(pcTaskName);
    // 进入一个无限循环，停止任务执行
    for (;;)
        ;
}