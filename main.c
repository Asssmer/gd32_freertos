#include "./454software/454software.h"
#include "./454software/sdcard.h"
//-----------------------------------------------------------------------
//                       底层驱动信息
//-----------------------------------------------------------------------
extern volatile MotorStatus motor_status; // 电机
extern volatile SensorData sensor_data;   // ADC(包含电磁阀状态,氧气压力,氧气浓度)
extern int16_t temperature_P7;              // P7_get();温度
extern int16_t temperature_P9;              // P9_get();温度
extern int16_t pressure_P10;                // P10_get();压力
extern int16_t pressure_P11;                // P11_get();压力
extern int16_t pressure_P12;                // P12_get();
extern uint16_t flow_P13;                    // P13_get();流量
extern uint16_t flow_P14;                    // P14_get();流量
extern uint16_t flow_P15;                    // P15_get();流量
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

//! 余辉教授文档需求
extern volatile struct struRtSample RtSample;
extern SemaphoreHandle_t RtSample_Mutex;
extern SemaphoreHandle_t iicCollect_Ready_Semaphore;
extern SemaphoreHandle_t iicCollect_Start_Semaphore;

//-----------------------------------------------------------------------
//                       任务函数原型
//-----------------------------------------------------------------------
void Task1(void *pvParameters);
void Task2(void *pvParameters);
void iicTask(void *pvParameters);
void gTaskRtSample(void *pvParameters);
void gTaskRtMotor(void *pvParameters);
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

    usart0_send_454("initinitialize finished!\n",10);

    // 初始化互斥量和信号量
    RtSample_Mutex = xSemaphoreCreateMutex();
    iicCollect_Ready_Semaphore = xSemaphoreCreateBinary();
    iicCollect_Start_Semaphore = xSemaphoreCreateBinary();

    motor_control(0);
    delay_ms_454(100);

    //! Heap_Size       EQU     0x00019000---->100KB
    // 创建3个 IIC 采集任务
    xTaskCreate(iicTask, "IIC1_Task", 1024, (void *)1, 50, NULL);
    xTaskCreate(iicTask, "IIC0_Task", 1024, (void *)2, 50, NULL);
    xTaskCreate(iicTask, "IIC2_Task", 1024, (void *)3, 50, NULL);

    // xTaskCreate(gTaskRtSample, "gTaskRtSample", 1024, (void *)1, 40, NULL);
    // xTaskCreate(gTaskRtMotor, "gTaskRtMotor", 1024, (void *)1, 40, NULL);
    // xTaskCreate(gTaskLocalRecSave, "gTaskLocalRecSave", 1024, (void *)1, 10, NULL);

    xTaskCreate(Task1, "test", 1024, (void *)1, 50, NULL);
    xTaskCreate(Task2, "led_toggle", 1024, (void *)1, 50, NULL);
    // 启动调度器
    vTaskStartScheduler();
    for (;;)
        ;
    return 0;
}
//-----------------------------------------------------------------------
//                       任务实现
//-----------------------------------------------------------------------
void Task2(void *pvParameters)
{
    while (1)
    {
        printf("Task2!\n");
        gpio_bit_toggle(GPIOG, GPIO_PIN_6);
        vTaskDelay(1000);
    }
}
void Task1(void *pvParameters)
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
        printf("Task1!\n");
        gpio_bit_toggle(GPIOG, GPIO_PIN_7);
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
            RtSample.uT1 = P7_get();
            RtSample.uT2 = P9_get();
            RtSample.uT3 = motor_status.motor_temperature;
            RtSample.u16P1 = pressure_P10;
            RtSample.u16P2 = pressure_P11;
            RtSample.u16P3 = pressure_P12;
            RtSample.u16F1 = flow_P13;
            RtSample.u16F2 = flow_P14;
            RtSample.u16F3 = flow_P15;
            RtSample.uMotor = motor_status.current_speed;
            RtSample.uO2 = sensor_data.oxygen;
            RtSample.bValve1 = sensor_data.p4_valve;
            RtSample.bValve2 = sensor_data.p5_valve;
            RtSample.bValve3 = sensor_data.p6_valve;
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

        // 3. 生成状态报文并放入发送队列(直接printf发送即可,上位机去解析字符串)
        printf("Speed: %d, Valve1: %f, Valve2: %f, Valve3: %f\n",
               motor_status.current_speed, sensor_data.p4_valve, sensor_data.p5_valve, sensor_data.p6_valve);
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