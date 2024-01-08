#include "./454software/454software.h"
//-----------------------------------------------------------------------
//                       �ײ�������Ϣ
//-----------------------------------------------------------------------
// ���
extern volatile MotorStatus motor_status;
// ADC
extern volatile SensorData sensor_data;
// �¶�
float temperature_P7; // P7_get();
float temperature_P9; // P9_get();
// ѹ��
float pressure_P10; // P10_get();
float pressure_P11; // P11_get();
float pressure_P12; // P12_get();
// ����
float flow_P13;
float flow_P14;
float flow_P15;
// LED����
//   LED1(ON),LED1(OFF)
//   LED2(ON),LED2(OFF)
//   LED3(ON),LED3(OFF)
//  ��ŷ�����
//   P4_PWM_set(0);
//   P5_PWM_set(0);
//   P6_PWM_set(0);
// ѹ�緧Ƭ����
//   YDP_control(ON),YDP_control(OFF)
// ���ӿ�������
//   Pulverizer_control(ON),Pulverizer_control(OFF),
//-----------------------------------------------------------------------
//                       ������ԭ��
//-----------------------------------------------------------------------
void Task1(void *pvParameters);
void Task2(void *pvParameters);
//-----------------------------------------------------------------------
//                       ������
//-----------------------------------------------------------------------
int main()
{
    init_454();
    printf("initinitialize finished!\n");

    P4_PWM_set(0);
    P5_PWM_set(0);
    P6_PWM_set(0);
    motor_control(0);
    delay_ms_454(100);
    //! Heap_Size       EQU     0x00019000---->100KB
    xTaskCreate(Task1, "sensor_get", 1024, (void *)1, 200, NULL);
    xTaskCreate(Task2, "speed_set", 1024, (void *)1, 200, NULL);
    // ����������
    vTaskStartScheduler();
    for (;;)
        ;
    return 0;
}
//-----------------------------------------------------------------------
//                       ����ʵ��
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
    while (1)
    {
        printf("����Task1!\n");
        gpio_bit_toggle(GPIOG, GPIO_PIN_7);
        vTaskDelay(1000);
    }
    // motor_pressure_flow(0, 80);
    // while (1)
    // {

    //     temperature_P7 = P7_get();
    //     temperature_P9 = P9_get();
    //     pressure_P10 = P10_get();
    //     pressure_P11 = P11_get();
    //     pressure_P12 = P12_get();
    //     flow_P13 = P13_get();
    //     flow_P14 = P14_get();
    //     flow_P15 = P15_get();
    //     printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    //            motor_status.current_speed,
    //            motor_status.motor_temperature,
    //            temperature_P7,
    //            temperature_P9,
    //            pressure_P10,
    //            pressure_P11,
    //            pressure_P12,
    //            flow_P13,
    //            flow_P14,
    //            flow_P15);
    // }
}
//-----------------------------------------------------------------------
//                       HOOK
//-----------------------------------------------------------------------
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // ��ӡ������ջ�������������
    printf("\nStack overflow in task: ");
    printf(pcTaskName);
    // ����һ������ѭ����ֹͣ����ִ��
    for (;;)
        ;
}