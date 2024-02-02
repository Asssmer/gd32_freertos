#ifndef RTOS_LEVEL_WANGJIAJUN
#define RTOS_LEVEL_WANGJIAJUN

#include "./454software.h"

//! 此文件为业务层相关代码的头文件

//-----------------------------------------------------------------------
//                       全局数据声明
//-----------------------------------------------------------------------
#define SAMPLE_RATE 10                        //! 指定是100
#define SAMPLES_PER_MINUTE (60 * SAMPLE_RATE) // 每分钟采样数据缓存数量

// 表1 - 病人类型
typedef enum
{
    PatientType_Adult = 1, // 成人
    PatientType_Neonate,   // 新生儿
    PatientType_Child,     // 儿童
    PatientType_Other      // 其他（默认成人）
} PatientType;

// 表2 - 病人传感器位置
typedef enum
{
    PatientSensorPosition_PatientEnd = 1, // 病人端
    PatientSensorPosition_Peripheral,     // 末梢部
    PatientSensorPosition_None,           // 外部无
    PatientSensorPosition_Other           // 其他（默认无）
} PatientSensorPosition;

// 表3 - 气体类型
typedef enum
{
    GasStandard_ATPD = 1, // ATPD
    GasStandard_STPD,     // STPD
    GasStandard_BTPS,     // BTPS
    GasStandard_Other     // 其他（默认ATPD）
} GasStandard;

// 表4 - 传感器类型
typedef enum
{
    SensorType_PressureOnly = 1,        // 只有压力
    SensorType_SensatronicAdult,        // Sensatronic 成人
    SensorType_SensatronicChild,        // Sensatronic 儿童
    SensorType_EnvitecSpiroQuantHAdult, // Envitec Spiro Quant H 成人
    SensorType_HamiltonNeonate,         // Hamilton 新生儿
    SensorType_RespironicsNeonate,      // Respironics 新生儿
    SensorType_SensatronicNeonate,      // Sensatronic 新生儿
    SensorType_Other                    // 其他（默认为只有压力）
} SensorType;

// 表5 - 需使能、真假判断对象
typedef enum
{
    EnableState_True = 1, // 真
    EnableState_False     // 假
} EnableState;

// 表6 - 操作模式
typedef enum
{
    OperationMode_Shutdown = 1, // 关机
    OperationMode_SelfTest,     // 自检
    OperationMode_Idle,         // 空闲
    OperationMode_Ventilation,  // 通气
    OperationMode_Service,      // 服务
    OperationMode_Zeroing,      // 归零
    OperationMode_Calibration,  // 校准
    OperationMode_Other         // 其他（默认关机）
} OperationMode;

// 表7 - 氧气浓度传感器类型
typedef enum
{
    OxygenSensorType_NotInstalled = 1, // 未安装
    OxygenSensorType_Analog,           // 模拟
    OxygenSensorType_Digital           // 数字
} OxygenSensorType;

// 表8 - 通气模式
typedef enum
{
    MOD_Standby = 0,
    MOD_SPN_CPAP,
    MOD_PC_CFLOW,
    MOD_PC_CMV,
    MOD_PC_ACV,
    MOD_PC_SIMV,
    MOD_PC_SIMV_Plus,
    MOD_PC_AMV,
    MOD_VC_CMV,
    MOD_VC_SIMV,
    MOD_VC_ACV,
    MOD_PC_APRV,
    MOD_PC_MMV
} VentilationMode;

// 表9 - 管路类型
typedef enum
{
    TubeType_Endotracheal = 1, // Endo气管管路
    TubeType_Tracheostomy      // 气管造口管路
} TubeType;

// 表10 - 湿化器类型
typedef enum
{
    HumidifierType_NotInstalled = 1, // 未安装
    HumidifierType_Enabled,          // 启用
    HumidifierType_HMEFilter         // HME过滤器
} HumidifierType;

// 表11 - 漏气补偿
typedef enum
{
    LeakageCompensationType_Off = 1,  // 关闭
    LeakageCompensationType_Manual,   // 手动
    LeakageCompensationType_Automatic // 自动
} LeakageCompensationType;

//__packed关键字来确保结构体不会有任何填充
typedef struct __packed
{
    uint16_t uIndex;     // 本分钟内序号，有效范围0~35999
    int16_t s16P1;       // 10pa
    int16_t s16P2;       // 10pa
    int16_t s16P3;       // 10pa
    uint16_t u16F1;      // 10ml/分钟
    uint16_t u16F2;      // 10ml/分钟
    uint16_t u16F3;      // 10ml/分钟
    uint8_t uO2;         // 1%  建议将uO2移到末尾
    int16_t s16T1;       // 0.1摄氏度
    int16_t s16T2;       // 0.1摄氏度
    int16_t s16T3;       // 0.1摄氏度
    uint32_t u16Motor;   //
    uint16_t u16MotorSV; // 通过PID计算后目标速度
    uint8_t uValve1;     // 调控O2
    uint8_t uValve2;     // 电磁阀P？
    uint8_t uValve3;     // 电磁阀P？
    uint8_t uYDPValve;   // 呼气阀门

} struRtSample;
struct stru10HzSamplePerMinute
{
    uint8_t u8Year, u8Month, u8Day, u8Hour, u8Minute; // 当前记录时间
    uint16_t u16P1[SAMPLES_PER_MINUTE], u16P2[SAMPLES_PER_MINUTE], u16P3[SAMPLES_PER_MINUTE];
    uint16_t u16F1[SAMPLES_PER_MINUTE], u16F2[SAMPLES_PER_MINUTE], u16F3[SAMPLES_PER_MINUTE];
};

struct struTarget
{
    uint8_t uVentilationMode;                        // 辅助通气模式
    uint16_t uInspirationTime;                       // 目标吸气持续时长（单位ms）
    uint16_t uExpirationTime;                        // 目标呼气持续时长（单位ms）
    uint16_t uPressureRiseTime;                      // 压力上升时长（单位ms）
    uint16_t uApnoeaTimeOut;                         // 呼吸暂停时长
    uint16_t uMaximumPressure;                       // 通气压力控制上限
    uint16_t uInspirationDeltaPressure;              // 吸气压力波动限, 用于智能判断自主吸气开始
    uint16_t uPositiveEndExpirationPressure;         // 呼气正压通气压力（压力控制主要指标）
    uint16_t uPressureSupportLowDeltaPressure;       // 吸气压力波动下限, 用于智能判断是否面罩脱落
    uint16_t uResuscitationSensitivityLevel;         // 复苏敏感级别
    uint16_t uTidalVolume;                           // 目标潮气量（单位ml）
    uint16_t uMaximumPressurePCVR;                   // PCVR最大压力
    uint16_t uMinimumPressurePCVR;                   // PCVR最小压力
    bool bIntrinsicPeepEnable;                       // 内部PEEP使能
    uint16_t uPercentageO2;                          // 氧浓度百分比
    uint16_t uContinuousFlow;                        // 持续流量
    uint16_t uOxygenSupplyConcentration;             // 氧供应浓度
    uint16_t uInspiratoryTriggerFlow;                // 吸气触发流
    uint16_t uExpiratoryFlowCycledTriggerPercentage; // 呼气流量循环触发百分比
    bool bPCVREnable;                                // PCVR使能
    bool bNonInvasiveVentilationEnable;              // 非侵入式通气使能
    bool bPressureSupportLowEnable;                  // 低压支持使能
    bool bInspiratoryFlowTriggerEnable;              // 吸气流量触发使能
    bool bLPOEnable;                                 // LPO使能
    bool bSighEnable;                                // 叹息使能
    bool bTubeCompensationInspirationEnable;         // 管路补偿吸气使能
    bool bP01Enable;                                 // P01使能
    uint16_t uRecruitmentInspirationDeltaPressure;   // 招募吸气压力差
    uint16_t uRecruitmentPEEP;                       // 招募PEEP
    uint16_t uRecruitmentNumberofBreaths;            // 招募呼吸次数
    bool bNebulizerEnable;                           // 雾化器使能
    uint16_t uSighDeltaPressure;                     // 叹息压力差
    uint16_t uSighDeltaTidalVolume;                  // 叹息潮气量差
    uint16_t uSighBreathCount;                       // 叹息呼吸计数
    bool bInspiratoryHoldEnable;                     // 吸气保持使能
    bool bRecruitmentEnable;                         // 招募使能
    bool bExpiratoryHoldEnable;                      // 呼气保持使能
    uint8_t uPatientType;                            // 患者类型
    uint8_t uPatientSensorPosition;                  // 患者传感器位置
    uint8_t uExpirationValvePosition;                // 呼气阀位置
    uint8_t uPatientSensorTemperature;               // 患者传感器温度上限
    bool bCompensateCompressibleVolumeEnable;        // 可压缩体积补偿使能
    uint8_t uTubeType;                               // 管路类型
    uint8_t uTubeDiameter;                           // 管路直径
    uint8_t TubeCompensationPercentage;              // 管路补偿百分比
    bool bTubeCompensationExpirationEnable;          // 管路补偿呼气使能
    uint16_t uNebulizerActiveTime;                   // 雾化器活动时间
    uint8_t uPatientSensorType;                      // 患者传感器类型
    uint8_t uGasStandard;                            // 气体标准
    uint8_t uHumidifierType;                         // 加湿器类型
    uint8_t uSensitivitySpontaneousDetection;        // 自发性检测敏感性
    uint16_t uMinuteVolume;                          // 分钟通气量
    uint16_t uInspiratoryFlow;                       // 吸气流量
    uint8_t uInspiratorySpontaneousBreathingWindow;  // 吸气自主呼吸窗
    uint8_t uExpiratorySpontaneousBreathingWindow;   // 呼气自主呼吸窗
    bool bInspiratoryPressureTriggerEnable;          // 吸气压力触发使能
    uint16_t uInspiratoryTriggerDeltaPressure;       // 吸气触发压力差
    int16_t sExpiratoryTriggerFlow;                  // 呼气触发流
    uint16_t uPressureSupportMaxStrokeTime;          // 压力支持最大中风时间
    uint16_t uInspiratoryHoldTime;                   // 吸气保持时间
    uint8_t uLeakageCompensationType;                // 漏气补偿类型
    uint8_t uLeakageCompensationPercentage;          // 漏气补偿百分比
    bool bPressureSupportHighEnable;                 // 高压支持使能
    uint16_t uPressureSupportHighDeltaPressure;      // 高压支持压力差
    bool bApnoeaDetectionEnable;                     // 呼吸暂停检测使能
    uint16_t uRecruitmentInspirationTime;            // 招募吸气时间
    uint16_t uRecruitmentExpirationTime;             // 招募呼气时间
    uint16_t uRecruitmentPressureRiseTime;           // 招募压力上升时间
    uint16_t uNebulizerExternallyDeliveredFlow;      // 外部送气流量
    uint8_t uNebulizerPosition;                      // 雾化器位置
    bool bPPSEnable;                                 // PPS使能
    uint8_t uPPSPercentageSupport;                   // PPS百分比支持
    uint16_t uPPSCompliance;                         // PPS顺应性
    uint16_t uPPSResistance;                         // PPS阻力
    uint16_t uExpiratoryFlowTriggerEnable;           // 呼气流量触发使能
    uint16_t uExpiratoryFlowCycledTriggerEnable;     // 呼气流量循环触发使能
};

typedef struct struRtMeasure
{
    uint16_t uPatientFlow;
    uint16_t uPatientPressure;
    uint16_t uOutputPressure;
    uint16_t uTrachealPressure;
    uint16_t uOutputFlow;
    uint16_t uAirFlow;
    uint16_t uOxygenFlow;
    uint16_t uPeakInspiratoryFlow;
    uint16_t uPeakExpiratoryFlow;
    uint16_t uPositiveEndExpiratoryPressure;
    uint16_t uPeakInspiratoryPressure;
    uint16_t uMeanPressure;
    uint16_t uMinimumPressure;
    uint16_t uP01Pressure;
    uint16_t uIntrinsicPeep;
    uint16_t uExpiratoryTidalVolumeSp;
    uint16_t uExpiratoryTidalVolumeMand;
    uint16_t uExpiratoryMinuteVolumeSp;
    uint16_t uExpiratoryMinuteVolumeMand;
    uint16_t uRespiratoryRateSp;
    uint16_t uRespiratoryRateMand;
    uint16_t uStrokewiseFiO2;
    uint16_t uLungResistance;
    uint16_t uDynamicCompliance;
    uint16_t uRSBI;
    uint16_t uPressureTimeProduct;
    uint16_t uLeakageVolume;
    uint16_t uAmbientPressure;
    uint16_t uOxygenSupplyPressure;
    uint16_t uGasInputTemperature;
    uint16_t uGasOutputTemperature;
    uint16_t uStandbyTime;
    uint16_t uRunningTime;
    uint16_t uPatientVolume;
    uint16_t uPlateauPressure;
    uint16_t uRealtimeFiO2;
    uint16_t uInspiratoryTidalVolumeMand;
    uint16_t uInspiratoryMinuteVolumeMand;
    uint16_t uInspiratoryTidalVolumeSp;
    uint16_t uInspiratoryMinuteVolumeSp;
    uint16_t uLeakageFlow;
    uint16_t uInspiratoryTime;
    uint16_t uExpiratoryTime;
    uint16_t uRCInspiratoryTime;
    uint16_t uRCExpiratoryTime;
    uint16_t uCompressibleVolume;
    uint16_t uExpiratoryMinuteVolumeTotal;
    uint16_t uRespiratoryRateTotal;
} struRtMeasure;

//-----------------------------------------------------------------------
//                       与上位机通信协议
//-----------------------------------------------------------------------

//! 上位机到板卡↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
#define DOWN_FRAME_HEADER_BYTE_1 0xEB
#define DOWN_FRAME_HEADER_BYTE_2 0x90
#define DOWN_FRAME_HEADER_BYTE_3 0xAA

void processDataFrame(char *frame, uint16_t length);
uint16_t calculate_checksum(uint8_t *data, uint16_t length);

void handleInstruction01(char *data, uint16_t length);
void handleInstruction02(char *data, uint16_t length);
void handleInstruction03(char *data, uint16_t length);
void handleInstruction10(char *data, uint16_t length);
void handleInstruction11(char *data, uint16_t length);

//! 板卡到上位机↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
#define UP_FRAME_HEADER_BYTE_1 0xEB
#define UP_FRAME_HEADER_BYTE_2 0x91
#define UP_FRAME_HEADER_BYTE_3 0xAA

void upload_data_frame(uint8_t instruction_type, void *data, uint16_t data_length);
void upload_struRtSample(void);

//-----------------------------------------------------------------------
//                       测试日志输出
//-----------------------------------------------------------------------
void motor_pressure_flow(int min_speed, int max_speed);
void RTOS_motor_pressure_flow(int min_speed, int max_speed);

#endif