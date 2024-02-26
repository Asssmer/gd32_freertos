#include "./454software/rtos_level.h"
//-----------------------------------------------------------------------
//                       �ײ�������Ϣ
//-----------------------------------------------------------------------
extern volatile MotorStatus motor_status; // ���
extern volatile SensorData sensor_data;   // ADC(������ŷ�״̬,����ѹ��,����Ũ��)
extern int16_t temperature_P7;            // P7_get();�¶�
extern int16_t temperature_P9;            // P9_get();�¶�
extern int16_t pressure_P10;              // P10_get();ѹ��
extern int16_t pressure_P11;              // P11_get();ѹ��
extern int16_t pressure_P12;              // P12_get();
extern uint16_t flow_P13;                 // P13_get();����
extern uint16_t flow_P14;                 // P14_get();����
extern uint16_t flow_P15;                 // P15_get();����
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

//-----------------------------------------------------------------------
//                       rtos_level
//-----------------------------------------------------------------------
//! ��Խ����ĵ�����
extern QueueHandle_t USART0_RX_xQueue;
extern volatile struRtSample RtSample;
extern SemaphoreHandle_t RtSample_Mutex;
extern SemaphoreHandle_t iicCollect_Ready_Semaphore;
extern SemaphoreHandle_t iicCollect_Start_Semaphore;

//-----------------------------------------------------------------------
//                       ������ԭ��
//-----------------------------------------------------------------------
void Test_report(void *pvParameters);
void led_toggle(void *pvParameters);
void iicTask(void *pvParameters);
void gTaskRtSample(void *pvParameters);
void gTaskRtMotor(void *pvParameters);
void gTaskCommMonitor(void *pvParameters);
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

    // ��ʼ�����������ź���
    RtSample_Mutex = xSemaphoreCreateMutex();
    iicCollect_Ready_Semaphore = xSemaphoreCreateBinary();
    iicCollect_Start_Semaphore = xSemaphoreCreateBinary();
    USART0_RX_xQueue = xQueueCreate(256, sizeof(char));

    motor_control(0);
    delay_ms_454(100);

    //! Heap_Size       EQU     0x00019000---->100KB
    // ����3�� IIC �ɼ�����

    // xTaskCreate(iicTask, "IIC1_Task", 1024, (void *)1, 50, NULL);
    // xTaskCreate(iicTask, "IIC0_Task", 1024, (void *)2, 50, NULL);
    // xTaskCreate(iicTask, "IIC2_Task", 1024, (void *)3, 50, NULL);

    // xTaskCreate(gTaskRtSample, "gTaskRtSample", 1024, (void *)1, 40, NULL);
    // xTaskCreate(gTaskRtMotor, "gTaskRtMotor", 1024, (void *)1, 40, NULL);
    // xTaskCreate(gTaskCommMonitor, "gTaskCommMonitor", 1024, (void *)1, 50, NULL);
    // xTaskCreate(gTaskLocalRecSave, "gTaskLocalRecSave", 1024, (void *)1, 10, NULL);

    // xTaskCreate(Test_report, "Test_report", 1024, (void *)1, 40, NULL);
    xTaskCreate(led_toggle, "led_toggle", 1024, (void *)1, 40, NULL);
    // ����������
    vTaskStartScheduler();
    for (;;)
        ;
    return 0;
}
//-----------------------------------------------------------------------
//                       ����ʵ��
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

        // 3. ����״̬���Ĳ����뷢�Ͷ���
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
            // ��������֡
            USART0_RX_frameBuffer[USART0_RX_bufferIndex++] = rxByte;
            // ����Ƿ��յ�������֡ͷ
            if (USART0_RX_bufferIndex >= 3)
            {
                if (USART0_RX_frameBuffer[0] == 0xEB && USART0_RX_frameBuffer[1] == 0x90 && USART0_RX_frameBuffer[2] == 0xAA)
                {
                    uint8_t dataLength = USART0_RX_frameBuffer[3];
                    // ����Ƿ��յ�������������֡
                    if (USART0_RX_bufferIndex >= (dataLength + 4))
                    {
                        // ��������֡
                        processDataFrame(USART0_RX_frameBuffer, USART0_RX_bufferIndex);
                        // ���û�����������Ϊ������֡��׼��
                        USART0_RX_bufferIndex = 0;
                    }
                }
                else
                {
                    // ���֡ͷ��ƥ�䣬���û���������������λ�ÿ�ʼ���
                    // ���֡ͷ��ƥ�䣬����ζ�ŵ�ǰ���յ������ݲ�����Ч��֡��ʼ����˳���ͨ���Ƴ���һ���ֽ���������һ�����ܵ�֡ͷ
                    memmove(USART0_RX_frameBuffer, USART0_RX_frameBuffer + 1, --USART0_RX_bufferIndex);
                }
            }
            // ��黺�����Ƿ�����
            if (USART0_RX_bufferIndex == 256)
            {
                // �������������Ҫ���д�����
                USART0_RX_bufferIndex = 0; // ���û���������
            }
        }
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