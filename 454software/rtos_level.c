#include "./rtos_level.h"
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
//-----------------------------------------------------------------------
//                       ȫ������
//-----------------------------------------------------------------------
SemaphoreHandle_t RtSample_Mutex;
SemaphoreHandle_t iicCollect_Ready_Semaphore;
SemaphoreHandle_t iicCollect_Start_Semaphore;

volatile struRtSample RtSample;
// �����б��ö�ٽ��б�������
PatientType patientType = PatientType_Adult;                                    // Ĭ�ϳ���
PatientSensorPosition patientSensorPosition = PatientSensorPosition_PatientEnd; // Ĭ�ϲ��˶�
GasStandard gasStandard = GasStandard_ATPD;                                     // Ĭ��ATPD
SensorType sensorType = SensorType_PressureOnly;                                // Ĭ��ֻ��ѹ��
EnableState enableState = EnableState_True;                                     // Ĭ��Ϊ��
OperationMode operationMode = OperationMode_Shutdown;                           // Ĭ�Ϲػ�
OxygenSensorType oxygenSensorType = OxygenSensorType_NotInstalled;              // Ĭ��δ��װ
VentilationMode ventilationMode = MOD_Standby;                                  // Ĭ��MOD_Standby
TubeType tubeType = TubeType_Endotracheal;                                      // Ĭ��Endo���ܹ�·
HumidifierType humidifierType = HumidifierType_NotInstalled;                    // Ĭ��δ��װ
LeakageCompensationType leakageCompensationType = LeakageCompensationType_Off;  // Ĭ�Ϲر�

//-----------------------------------------------------------------------
//                       ��λ��ͨ��Э��
//-----------------------------------------------------------------------

// У��λ����
uint16_t calculate_checksum(uint8_t *data, uint16_t length)
{
    uint32_t checksum = 0;

    for (uint16_t i = 3; i < length - 2; ++i)
    {
        checksum += data[i];
    }

    // ȡ��16λ
    return (uint16_t)(checksum & 0xFFFF);
}

//! ��λ�����忨����������������������������������������

// �������յ�������
void processDataFrame(char *frame, uint16_t length)
{
    // ������ʵ�ֶ�����֡�Ĵ���
    // ��ȡָ�����
    uint8_t instructionType = frame[4];
    // ����ָ�������
    switch (instructionType)
    {
    case 0x01:
    {
        // ָ��01�Ĵ���
        handleInstruction01(frame + 5, length - 3);
        break;
    }
    case 0x02:
    { // ָ��02�Ĵ���
        handleInstruction02(frame + 5, length - 3);
        break;
    }
    case 0x03:
    { // ָ��03�Ĵ���
        handleInstruction03(frame + 5, length - 3);
        break;
    }
    case 0x10:
    { // ָ��10�Ĵ���
        handleInstruction10(frame + 5, length - 3);
        break;
    }
    case 0x11:
    { // ָ��11�Ĵ���
        handleInstruction11(frame + 5, length - 3);
        break;
    }
    default:
    {
        break;
    }
    }
}

// ָ��01
void handleInstruction01(char *data, uint16_t length)
{
    // ȷ�����ݳ�����ȷ
    if (length < 18)
    { // ָ�����1Ӧ����18���ֽڵ�����
        // ���ݳ��Ȳ���ȷ�Ĵ�����
        return;
    }
    // ���������ֶ�
    PatientType patientType = data[0];                          // ��������
    PatientSensorPosition sensorPosition = data[1];             // ���˴�����λ��
    GasStandard gasStandard = data[2];                          // �����׼
    SensorType sensorType = data[3];                            // ����������
    EnableState externalMeasurement = data[4];                  // �ⲿ����ʹ��״̬
    OperationMode operationMode = data[5];                      // ����ģʽ
    OxygenSensorType oxygenSensorType = data[6];                // ����Ũ�ȴ���������
    VentilationMode ventilationMode = data[7];                  // ͨ��ģʽ
    uint8_t patientSensorTemperature = data[8];                 // ���˴������¶�
    TubeType tubeType = data[9];                                // ��·����
    uint8_t tubeDiameter = data[10];                            // ��·ֱ��
    uint8_t sensitivitySpontaneousDetection = data[11];         // �Է��������������
    HumidifierType humidifierType = data[12];                   // ʪ��������
    uint8_t nebulizerPosition = data[13];                       // ����λ��
    LeakageCompensationType leakageCompensationType = data[14]; // ©����������
    uint8_t safetyValueManualOpen = data[15];                   // ��ȫ���ֶ�����
    // ����ֻ��ʾ��������ʵ��ȡ����ϵͳ���ʹ����Щ����
    // updateSystemSettings(patientType, sensorPosition, gasStandard, sensorType, externalMeasurement, operationMode, oxygenSensorType, ventilationMode, patientSensorTemperature, tubeType, tubeDiameter, sensitivitySpontaneousDetection, humidifierType, nebulizerPosition, leakageCompensationType, safetyValueManualOpen);
}
// ָ��02
void handleInstruction02(char *data, uint16_t length)
{
    printf("handleInstruction02\n");
}
// ָ��03
void handleInstruction03(char *data, uint16_t length)
{
    printf("handleInstruction03\n");
}
// ָ��10
// ���ܣ���λ����ѯ�������ذ嵱ǰͨ��ģʽ����ģʽ�����õĵ��ز�����Ϣ���������ذ巵��ͨ��ģʽ����ģʽ�����õĵ��ز�����Ϣ
// ����λ�����������ذ巢�͵�10ָ�ָ������Ϊ�գ�����֡ʾ����EB90AA03100010
// �����ذ�Ӧ��10ָ�����֡ʾ��EB91AA0310010010
void handleInstruction10(char *data, uint16_t length)
{
    printf("handleInstruction10\n");
}
// ָ��11
// ���ܣ���λ�����ú������ذ嵱ǰͨ��ģʽ����ģʽ�ĵ��ز�����Ϣ���������ذ巵��ͨ��ģʽ����ģʽ�����õĵ��ز�����Ϣ
//  ����λ�����������ذ巢�͵�11ָ�ָ�����ݲμ�ָ��10������ݣ�����֡ʾ����EB90AAYY11..............YYYY
//  �����ذ�Ӧ��11ָ�����֡ʾ��EB91AAYY11..............YYYY
void handleInstruction11(char *data, uint16_t length)
{
    printf("handleInstruction11\n");
}

//! �忨����λ��������������������������������������

// ��������֡
void upload_data_frame(uint8_t instruction_type, void *data, uint16_t data_length)
{
    uint8_t data_frame[256]; // ����ʵ���������

    // ȷ�����ݳ��ȼ��Ϲ̶������ֽڲ��ᳬ��data_frame���ܴ�С
    if (data_length + 7 > sizeof(data_frame))
    {
        // ������ݼ��϶����ֽڳ����������С����������ض�����
        // ������Լ�¼���󡢽ض����ݻ��ȡ�����ָ���ʩ
        return; // ������ֱ�ӷ��أ���ִ�к�������
    }

    data_frame[0] = UP_FRAME_HEADER_BYTE_1;
    data_frame[1] = UP_FRAME_HEADER_BYTE_2;
    data_frame[2] = UP_FRAME_HEADER_BYTE_3;
    // ���ݳ��ȣ�������֡ͷ��֡β
    data_frame[3] = data_length + 2;
    // ָ�����
    data_frame[4] = instruction_type;
    // �������ݵ�����֡
    memcpy(&data_frame[5], data, data_length);
    // ����У��λ
    uint16_t checksum = calculate_checksum(data_frame, data_length + 5);
    // ���У��λ������֡
    data_frame[data_length + 5] = (uint8_t)(checksum & 0xFF);
    data_frame[data_length + 6] = (uint8_t)(checksum >> 8);
    // �������ݵ���λ��
    usart0_send_454(data_frame, data_length + 7);
}

// 1��ָ��01
// �������ذ嶨ʱ�����ϱ�������ʵʱ�ɼ�����(10Hz)��Ӧ���ݽṹ
void upload_struRtSample(void)
{
    upload_data_frame(0x01, &RtSample, sizeof(RtSample));
}
// 4��ָ��12
// �������ذ���������λ��ͨ�����κ������ڼ����
// ���ܣ��������ذ���������λ��ͨ�����κ������ڼ����
// �����ذ�����λ�����͵�12ָ�ָ������Ϊ���κ������ڼ����������֡ʾ����EB91AAYY12..............YYYY

//-----------------------------------------------------------------------
//                       ������־
//-----------------------------------------------------------------------
void motor_pressure_flow(int min_speed, int max_speed)
{
    if (min_speed < 0 || min_speed > 100 || max_speed < 0 || max_speed > 100)
    {
        printf("Error: Speed values must be between 0 and 100 and min_speed should be less than or equal to max_speed.\n");
        return;
    }
    int i, j;
    // �¶�
    float temperature_P7_temp;
    float temperature_P9_temp;
    // ѹ��
    float pressure_P10_temp;
    float pressure_P11_temp;
    float pressure_P12_temp;
    // ����
    float flow_P13_temp;
    float flow_P14_temp;
    float flow_P15_temp;
    TickType_t startTicks, endTicks, durationTicks;
    TickType_t currentTicks = xTaskGetTickCount();

    printf("motor_speed_percent,motor_speed,motor_temp,temperature_P7,temperature_P9,pressure_P10,pressure_P11,pressure_P12,flow_P13,flow_P14,flow_P15,diff_time\n");

    for (i = min_speed; i <= max_speed; i++)
    {
        startTicks = xTaskGetTickCount();
        motor_speed_percent(i);

        for (j = 0; j < 20; j++)
        {

            temperature_P7_temp = P7_get();
            temperature_P9_temp = P9_get();
            pressure_P10_temp = P10_get();
            pressure_P11_temp = P11_get();
            pressure_P12_temp = P12_get();
            flow_P13_temp = P13_get();
            flow_P14_temp = P14_get();
            flow_P15_temp = P15_get();
            endTicks = xTaskGetTickCount();
            durationTicks = endTicks - startTicks;
            printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u\n",
                   i,
                   motor_status.current_speed,
                   motor_status.motor_temperature,
                   temperature_P7,
                   temperature_P9,
                   pressure_P10,
                   pressure_P11,
                   pressure_P12,
                   flow_P13,
                   flow_P14,
                   flow_P15,
                   durationTicks);
        }
        delay_s_454(10);
    }
    motor_speed_percent(0);
}
void RTOS_motor_pressure_flow(int min_speed, int max_speed)
{
    if (min_speed < 0 || min_speed > 100 || max_speed < 0 || max_speed > 100)
    {
        printf("Error: Speed values must be between 0 and 100 and min_speed should be less than or equal to max_speed.\n");
        return;
    }
    int i, j;
    // �¶�
    float temperature_P7_temp;
    float temperature_P9_temp;
    // ѹ��
    float pressure_P10_temp;
    float pressure_P11_temp;
    float pressure_P12_temp;
    // ����
    float flow_P13_temp;
    float flow_P14_temp;
    float flow_P15_temp;
    TickType_t startTicks, endTicks, durationTicks;
    TickType_t currentTicks = xTaskGetTickCount();

    printf("motor_speed_percent,motor_speed,motor_temp,temperature_P7,temperature_P9,pressure_P10,pressure_P11,pressure_P12,flow_P13,flow_P14,flow_P15,diff_time\n");

    for (i = min_speed; i <= max_speed; i++)
    {
        startTicks = xTaskGetTickCount();
        motor_speed_percent(i);

        for (j = 0; j < 20; j++)
        {
            xSemaphoreGive(iicCollect_Start_Semaphore);
            xSemaphoreTake(iicCollect_Ready_Semaphore, portMAX_DELAY);

            temperature_P7_temp = P7_get();
            temperature_P9_temp = P9_get();
            pressure_P10_temp = pressure_P10;
            pressure_P11_temp = pressure_P11;
            pressure_P12_temp = pressure_P12;
            flow_P13_temp = flow_P13;
            flow_P14_temp = flow_P14;
            flow_P15_temp = flow_P15;
            endTicks = xTaskGetTickCount();
            durationTicks = endTicks - startTicks;
            printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u\n",
                   i,
                   motor_status.current_speed,
                   motor_status.motor_temperature,
                   temperature_P7_temp,
                   temperature_P9_temp,
                   pressure_P10_temp,
                   pressure_P11_temp,
                   pressure_P12_temp,
                   flow_P13_temp,
                   flow_P14_temp,
                   flow_P15_temp,
                   durationTicks);
        }
        delay_s_454(3);
    }
    motor_speed_percent(0);
}
