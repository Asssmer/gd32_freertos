#include "./rtos_level.h"
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
//-----------------------------------------------------------------------
//                       全局数据
//-----------------------------------------------------------------------
SemaphoreHandle_t RtSample_Mutex;
SemaphoreHandle_t iicCollect_Ready_Semaphore;
SemaphoreHandle_t iicCollect_Start_Semaphore;

volatile struRtSample RtSample;
// 对所有表的枚举进行变量定义
PatientType patientType = PatientType_Adult;                                    // 默认成人
PatientSensorPosition patientSensorPosition = PatientSensorPosition_PatientEnd; // 默认病人端
GasStandard gasStandard = GasStandard_ATPD;                                     // 默认ATPD
SensorType sensorType = SensorType_PressureOnly;                                // 默认只有压力
EnableState enableState = EnableState_True;                                     // 默认为真
OperationMode operationMode = OperationMode_Shutdown;                           // 默认关机
OxygenSensorType oxygenSensorType = OxygenSensorType_NotInstalled;              // 默认未安装
VentilationMode ventilationMode = MOD_Standby;                                  // 默认MOD_Standby
TubeType tubeType = TubeType_Endotracheal;                                      // 默认Endo气管管路
HumidifierType humidifierType = HumidifierType_NotInstalled;                    // 默认未安装
LeakageCompensationType leakageCompensationType = LeakageCompensationType_Off;  // 默认关闭

//-----------------------------------------------------------------------
//                       上位机通信协议
//-----------------------------------------------------------------------

// 校验位计算
uint16_t calculate_checksum(uint8_t *data, uint16_t length)
{
    uint32_t checksum = 0;

    for (uint16_t i = 3; i < length - 2; ++i)
    {
        checksum += data[i];
    }

    // 取低16位
    return (uint16_t)(checksum & 0xFFFF);
}

//! 上位机到板卡↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

// 解析接收到的数据
void processDataFrame(char *frame, uint16_t length)
{
    // 在这里实现对数据帧的处理
    // 获取指令类别
    uint8_t instructionType = frame[4];
    // 根据指令类别处理
    switch (instructionType)
    {
    case 0x01:
    {
        // 指令01的处理
        handleInstruction01(frame + 5, length - 3);
        break;
    }
    case 0x02:
    { // 指令02的处理
        handleInstruction02(frame + 5, length - 3);
        break;
    }
    case 0x03:
    { // 指令03的处理
        handleInstruction03(frame + 5, length - 3);
        break;
    }
    case 0x10:
    { // 指令10的处理
        handleInstruction10(frame + 5, length - 3);
        break;
    }
    case 0x11:
    { // 指令11的处理
        handleInstruction11(frame + 5, length - 3);
        break;
    }
    default:
    {
        break;
    }
    }
}

// 指令01
void handleInstruction01(char *data, uint16_t length)
{
    // 确保数据长度正确
    if (length < 18)
    { // 指令类别1应该有18个字节的数据
        // 数据长度不正确的错误处理
        return;
    }
    // 解析数据字段
    PatientType patientType = data[0];                          // 病人类型
    PatientSensorPosition sensorPosition = data[1];             // 病人传感器位置
    GasStandard gasStandard = data[2];                          // 气体标准
    SensorType sensorType = data[3];                            // 传感器类型
    EnableState externalMeasurement = data[4];                  // 外部测量使能状态
    OperationMode operationMode = data[5];                      // 操作模式
    OxygenSensorType oxygenSensorType = data[6];                // 氧气浓度传感器类型
    VentilationMode ventilationMode = data[7];                  // 通气模式
    uint8_t patientSensorTemperature = data[8];                 // 病人传感器温度
    TubeType tubeType = data[9];                                // 管路类型
    uint8_t tubeDiameter = data[10];                            // 管路直径
    uint8_t sensitivitySpontaneousDetection = data[11];         // 自发呼吸检测灵敏度
    HumidifierType humidifierType = data[12];                   // 湿化器类型
    uint8_t nebulizerPosition = data[13];                       // 雾化器位置
    LeakageCompensationType leakageCompensationType = data[14]; // 漏气补偿类型
    uint8_t safetyValueManualOpen = data[15];                   // 安全阀手动开启
    // 这里只是示例，具体实现取决于系统如何使用这些数据
    // updateSystemSettings(patientType, sensorPosition, gasStandard, sensorType, externalMeasurement, operationMode, oxygenSensorType, ventilationMode, patientSensorTemperature, tubeType, tubeDiameter, sensitivitySpontaneousDetection, humidifierType, nebulizerPosition, leakageCompensationType, safetyValueManualOpen);
}
// 指令02
void handleInstruction02(char *data, uint16_t length)
{
    printf("handleInstruction02\n");
}
// 指令03
void handleInstruction03(char *data, uint16_t length)
{
    printf("handleInstruction03\n");
}
// 指令10
// 功能：上位机查询呼吸主控板当前通气模式及该模式下设置的调控参数信息，呼吸主控板返回通气模式及该模式下设置的调控参数信息
// ①上位机往呼吸主控板发送的10指令，指令内容为空，数据帧示例：EB90AA03100010
// ②主控板应答10指令，数据帧示例EB91AA0310010010
void handleInstruction10(char *data, uint16_t length)
{
    printf("handleInstruction10\n");
}
// 指令11
// 功能：上位机设置呼吸主控板当前通气模式及该模式的调控参数信息，呼吸主控板返回通气模式及该模式下设置的调控参数信息
//  ①上位机往呼吸主控板发送的11指令，指令内容参见指令10相关内容，数据帧示例：EB90AAYY11..............YYYY
//  ②主控板应答11指令，数据帧示例EB91AAYY11..............YYYY
void handleInstruction11(char *data, uint16_t length)
{
    printf("handleInstruction11\n");
}

//! 板卡到上位机↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

// 构造数据帧
void upload_data_frame(uint8_t instruction_type, void *data, uint16_t data_length)
{
    uint8_t data_frame[256]; // 根据实际情况调整

    // 确保数据长度加上固定额外字节不会超过data_frame的总大小
    if (data_length + 7 > sizeof(data_frame))
    {
        // 如果数据加上额外字节超过了数组大小，处理错误或截断数据
        // 这里可以记录错误、截断数据或采取其他恢复措施
        return; // 举例：直接返回，不执行后续操作
    }

    data_frame[0] = UP_FRAME_HEADER_BYTE_1;
    data_frame[1] = UP_FRAME_HEADER_BYTE_2;
    data_frame[2] = UP_FRAME_HEADER_BYTE_3;
    // 数据长度，不包括帧头和帧尾
    data_frame[3] = data_length + 2;
    // 指令类别
    data_frame[4] = instruction_type;
    // 拷贝数据到数据帧
    memcpy(&data_frame[5], data, data_length);
    // 计算校验位
    uint16_t checksum = calculate_checksum(data_frame, data_length + 5);
    // 添加校验位到数据帧
    data_frame[data_length + 5] = (uint8_t)(checksum & 0xFF);
    data_frame[data_length + 6] = (uint8_t)(checksum >> 8);
    // 发送数据到上位机
    usart0_send_454(data_frame, data_length + 7);
}

// 1、指令01
// 呼吸主控板定时主动上报传感器实时采集数据(10Hz)对应数据结构
void upload_struRtSample(void)
{
    upload_data_frame(0x01, &RtSample, sizeof(RtSample));
}
// 4、指令12
// 呼吸主控板主动向上位机通报本次呼吸周期检测结果
// 功能：呼吸主控板主动向上位机通报本次呼吸周期检测结果
// ①主控板向上位机发送的12指令，指令内容为本次呼吸周期检测结果，数据帧示例：EB91AAYY12..............YYYY

//-----------------------------------------------------------------------
//                       测试日志
//-----------------------------------------------------------------------
void motor_pressure_flow(int min_speed, int max_speed)
{
    if (min_speed < 0 || min_speed > 100 || max_speed < 0 || max_speed > 100)
    {
        printf("Error: Speed values must be between 0 and 100 and min_speed should be less than or equal to max_speed.\n");
        return;
    }
    int i, j;
    // 温度
    float temperature_P7_temp;
    float temperature_P9_temp;
    // 压力
    float pressure_P10_temp;
    float pressure_P11_temp;
    float pressure_P12_temp;
    // 流量
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
    // 温度
    float temperature_P7_temp;
    float temperature_P9_temp;
    // 压力
    float pressure_P10_temp;
    float pressure_P11_temp;
    float pressure_P12_temp;
    // 流量
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
