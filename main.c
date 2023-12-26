
#include "./454software/454software.h"

// 任务函数原型
void Task1(void *pvParameters);
void Task2(void *pvParameters);

// 电机
extern volatile MotorStatus motor_status;
// ADC
extern volatile uint16_t adc_values_454[ADC_CHANNEL_COUNT];
extern volatile SensorData sensor_data;
extern char usart0_res[12];
// 温度
float temperature_P7;
float temperature_P9;
// 压力
float pressure_P10;
float pressure_P11;
float pressure_P12;
// 流量
float flow_P13;
float flow_P14;
float flow_P15;

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

// 任务实现
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