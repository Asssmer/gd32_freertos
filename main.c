#include "./454software/454software.h"
#include "./454software/sdcard.h"
//-----------------------------------------------------------------------
//                       �ײ�������Ϣ
//-----------------------------------------------------------------------
extern volatile MotorStatus motor_status; // ���
extern volatile SensorData sensor_data;   // ADC(������ŷ�״̬,����ѹ��,����Ũ��)
extern int16_t temperature_P7;              // P7_get();�¶�
extern int16_t temperature_P9;              // P9_get();�¶�
extern int16_t pressure_P10;                // P10_get();ѹ��
extern int16_t pressure_P11;                // P11_get();ѹ��
extern int16_t pressure_P12;                // P12_get();
extern uint16_t flow_P13;                    // P13_get();����
extern uint16_t flow_P14;                    // P14_get();����
extern uint16_t flow_P15;                    // P15_get();����
// LED����
//   LED1(ON),LED1(OFF)
//   LED2(ON),LED2(OFF)
//   LED3(ON),LED3(OFF)
//  ��ŷ�����
//   P4_PWM_set(0);-->����0-100
//   P5_PWM_set(0);-->����0-100
//   P6_PWM_set(0);-->����0-100
// ѹ�緧Ƭ����
//   YDP_control(ON),YDP_control(OFF)
// ���ӿ�������
//   Pulverizer_control(ON),Pulverizer_control(OFF),

//! ��Խ����ĵ�����
extern volatile struct struRtSample RtSample;
extern SemaphoreHandle_t RtSample_Mutex;
extern SemaphoreHandle_t iicCollect_Ready_Semaphore;
extern SemaphoreHandle_t iicCollect_Start_Semaphore;

//-----------------------------------------------------------------------
//                       ������ԭ��
//-----------------------------------------------------------------------
void Task1(void *pvParameters);
void Task2(void *pvParameters);
void iicTask(void *pvParameters);
void gTaskRtSample(void *pvParameters);
void gTaskRtMotor(void *pvParameters);
void gTaskLocalRecSave(void *pvParameters);
//-----------------------------------------------------------------------
//                       ������
//-----------------------------------------------------------------------
int main()
{

    init_454();
    // SD_init_454();
    printf("initinitialize finished!\n");
    // ��ȡУ׼����
    // align_data();

    usart0_send_454("initinitialize finished!\n",10);

    // ��ʼ�����������ź���
    RtSample_Mutex = xSemaphoreCreateMutex();
    iicCollect_Ready_Semaphore = xSemaphoreCreateBinary();
    iicCollect_Start_Semaphore = xSemaphoreCreateBinary();

    motor_control(0);
    delay_ms_454(100);

    //! Heap_Size       EQU     0x00019000---->100KB
    // ����3�� IIC �ɼ�����
    xTaskCreate(iicTask, "IIC1_Task", 1024, (void *)1, 50, NULL);
    xTaskCreate(iicTask, "IIC0_Task", 1024, (void *)2, 50, NULL);
    xTaskCreate(iicTask, "IIC2_Task", 1024, (void *)3, 50, NULL);

    // xTaskCreate(gTaskRtSample, "gTaskRtSample", 1024, (void *)1, 40, NULL);
    // xTaskCreate(gTaskRtMotor, "gTaskRtMotor", 1024, (void *)1, 40, NULL);
    // xTaskCreate(gTaskLocalRecSave, "gTaskLocalRecSave", 1024, (void *)1, 10, NULL);

    xTaskCreate(Task1, "test", 1024, (void *)1, 50, NULL);
    xTaskCreate(Task2, "led_toggle", 1024, (void *)1, 50, NULL);
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
        // ����ȴ��ź���״̬
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
        // �����ź�����֪ͨ��ʱ������Զ�ȡ����
        xSemaphoreGive(iicCollect_Ready_Semaphore);
    }
}

void gTaskRtSample(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / SAMPLE_RATE); // ���� SAMPLE_RATE ������ʱ

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        xSemaphoreGive(iicCollect_Start_Semaphore);
        xSemaphoreTake(iicCollect_Ready_Semaphore, portMAX_DELAY);
        // ��ȡ������
        if (xSemaphoreTake(RtSample_Mutex, (TickType_t)50) == pdTRUE)
        {
            // �ɼ����ݲ��������ݽṹ
            RtSample.uIndex = xLastWakeTime; // ֱ��ʹ�õ�ǰ��TickCount
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
            // �ͷŻ�����
            xSemaphoreGive(RtSample_Mutex);
        }
        else
        {
            // ��������������ȡʧ��
        }
    }
}
void gTaskRtMotor(void *pvParameters)
{
    // ��ʽ��״̬����
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 10); // 10Hz, ��ÿ100����ִ��һ��

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. ��ȡ���Ʋ��������Ƶ��
        // MotorControlParams motorControlParams = gAlgorithm.Model.CalcPID(&gData.Target);
        // motor_control(motorControlParams.speed);

        // 2. ����ȫ�����ݽṹ
        // ������gTaskRtSample�и���

        // 3. ����״̬���Ĳ����뷢�Ͷ���(ֱ��printf���ͼ���,��λ��ȥ�����ַ���)
        printf("Speed: %d, Valve1: %f, Valve2: %f, Valve3: %f\n",
               motor_status.current_speed, sensor_data.p4_valve, sensor_data.p5_valve, sensor_data.p6_valve);
    }
}
void gTaskLocalRecSave(void *pvParameters)
{
    // �����
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