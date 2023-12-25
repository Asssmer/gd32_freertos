#include "FreeRTOS.h"
#include "task.h"
#include "./454software/454software.h"

// 任务函数原型
void motor_speed_set(void *pvParameters);
void sensor_get(void *pvParameters);

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
    motor_control(300);

    xTaskCreate(sensor_get, "sensor_get", 1024, (void *)1, 15, NULL);
    xTaskCreate(motor_speed_set, "speed_set", 1024, (void *)1, 15, NULL);
    // 启动调度器
    vTaskStartScheduler();
    for (;;)
        ;
    return 0;
}

// 任务实现
void motor_speed_set(void *pvParameters)
{
    // int taskNumber = (int)pvParameters;
    // uint16_t motor_control_data = 0;
    // motor_control(0);
    // while (1)
    // {
    //     gpio_bit_toggle(GPIOG, GPIO_PIN_7);
    //     delay_ms_454(1000);
    // }
    // for (;;)
    // {
    //     if (scanf("%hu", &motor_control_data) == 1)
    //     {
    //         motor_control(motor_control_data);
    //         printf("Set motor speed:%hu\n", motor_control_data);
    //         motor_control_data = 0;
    //     }
    //     else
    //     {
    //         motor_control_data = 0;
    //     }
    // }
}

void sensor_get(void *pvParameters)
{
    int taskNumber = (int)pvParameters;
    for (;;)
    {
        temperature_P7 = P7_get();
        temperature_P9 = P9_get();
        pressure_P10 = P10_get();
        pressure_P11 = P11_get();
        pressure_P12 = P12_get();
        flow_P13 = P13_get();
        flow_P14 = P14_get();
        flow_P15 = P15_get();
        // gpio_bit_toggle(GPIOG, GPIO_PIN_6);
        // delay_ms_454(1000);

        printf("\nmotor_speed:%.2f\n", motor_status.current_speed);
        printf("\nmotor_temp:%.2f\n", motor_status.motor_temperature);
        printf("\ntemperature_P7:%.2f\n", temperature_P7);
        printf("\ntemperature_P9:%.2f\n", temperature_P9);
        printf("\npressure_P10:%.2f\n", pressure_P10);
        printf("\npressure_P11:%.2f\n", pressure_P11);
        printf("\npressure_P12:%.2f\n", pressure_P12);
        printf("\nflow_P13:%.2f\n", flow_P13);
        printf("\nflow_P14:%.2f\n", flow_P14);
        printf("\nflow_P15:%.2f\n", flow_P15);
        printf("\n-------------------------------");
    }
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