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
    P4_PWM_set(0);
    P5_PWM_set(0);
    P6_PWM_set(0);
    motor_control(0);
    delay_ms_454(100);
    //! Stack_Size      EQU     0x00019000---->100KB
    xTaskCreate(Task1, "sensor_get", 1024, (void *)1, 15, NULL);
    xTaskCreate(Task2, "speed_set", 1024, (void *)1, 15, NULL);
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
    // ��ӡ������ջ�������������
    printf("\nStack overflow in task: ");
    printf(pcTaskName);
    // ����һ������ѭ����ֹͣ����ִ��
    for (;;)
        ;
}