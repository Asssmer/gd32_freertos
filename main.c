#include "./454software/454software.h"
//-----------------------------------------------------------------------
//                       底层驱动信息
//-----------------------------------------------------------------------
// 电机
extern volatile MotorStatus motor_status;
// ADC
extern volatile SensorData sensor_data;
// 温度
float temperature_P7; // P7_get();
float temperature_P9; // P9_get();
// 压力
float pressure_P10; // P10_get();
float pressure_P11; // P11_get();
float pressure_P12; // P12_get();
// 流量
float flow_P13;
float flow_P14;
float flow_P15;
// LED控制
//   LED1(ON),LED1(OFF)
//   LED2(ON),LED2(OFF)
//   LED3(ON),LED3(OFF)
//  电磁阀控制
//   P4_PWM_set(0);
//   P5_PWM_set(0);
//   P6_PWM_set(0);
// 压电阀片控制
//   YDP_control(ON),YDP_control(OFF)
//-----------------------------------------------------------------------
//                       任务函数原型
//-----------------------------------------------------------------------
void Task1(void *pvParameters);
void Task2(void *pvParameters);
//-----------------------------------------------------------------------
//                       主函数
//-----------------------------------------------------------------------
int main()
{
    init_454();
    P4_PWM_set(0);
    P5_PWM_set(0);
    P6_PWM_set(0);
    motor_control(0);
    delay_ms_454(100);
    //! Stack_Size      EQU     0x00019000---->100KB
    xTaskCreate(Task1, "sensor_get", 1024, (void *)1, 15, NULL);
    xTaskCreate(Task2, "speed_set", 1024, (void *)1, 15, NULL);
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
    
}

void Task1(void *pvParameters)
{

    motor_pressure_flow(0, 80);

    // gpio_bit_toggle(GPIOG, GPIO_PIN_6);
    // delay_ms_454(1000);
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