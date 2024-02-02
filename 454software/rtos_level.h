#ifndef RTOS_LEVEL_WANGJIAJUN
#define RTOS_LEVEL_WANGJIAJUN

#include "./454software.h"

//! ���ļ�Ϊҵ�����ش����ͷ�ļ�

//-----------------------------------------------------------------------
//                       ȫ����������
//-----------------------------------------------------------------------
#define SAMPLE_RATE 10                        //! ָ����100
#define SAMPLES_PER_MINUTE (60 * SAMPLE_RATE) // ÿ���Ӳ������ݻ�������

// ��1 - ��������
typedef enum
{
    PatientType_Adult = 1, // ����
    PatientType_Neonate,   // ������
    PatientType_Child,     // ��ͯ
    PatientType_Other      // ������Ĭ�ϳ��ˣ�
} PatientType;

// ��2 - ���˴�����λ��
typedef enum
{
    PatientSensorPosition_PatientEnd = 1, // ���˶�
    PatientSensorPosition_Peripheral,     // ĩ�Ҳ�
    PatientSensorPosition_None,           // �ⲿ��
    PatientSensorPosition_Other           // ������Ĭ���ޣ�
} PatientSensorPosition;

// ��3 - ��������
typedef enum
{
    GasStandard_ATPD = 1, // ATPD
    GasStandard_STPD,     // STPD
    GasStandard_BTPS,     // BTPS
    GasStandard_Other     // ������Ĭ��ATPD��
} GasStandard;

// ��4 - ����������
typedef enum
{
    SensorType_PressureOnly = 1,        // ֻ��ѹ��
    SensorType_SensatronicAdult,        // Sensatronic ����
    SensorType_SensatronicChild,        // Sensatronic ��ͯ
    SensorType_EnvitecSpiroQuantHAdult, // Envitec Spiro Quant H ����
    SensorType_HamiltonNeonate,         // Hamilton ������
    SensorType_RespironicsNeonate,      // Respironics ������
    SensorType_SensatronicNeonate,      // Sensatronic ������
    SensorType_Other                    // ������Ĭ��Ϊֻ��ѹ����
} SensorType;

// ��5 - ��ʹ�ܡ�����ж϶���
typedef enum
{
    EnableState_True = 1, // ��
    EnableState_False     // ��
} EnableState;

// ��6 - ����ģʽ
typedef enum
{
    OperationMode_Shutdown = 1, // �ػ�
    OperationMode_SelfTest,     // �Լ�
    OperationMode_Idle,         // ����
    OperationMode_Ventilation,  // ͨ��
    OperationMode_Service,      // ����
    OperationMode_Zeroing,      // ����
    OperationMode_Calibration,  // У׼
    OperationMode_Other         // ������Ĭ�Ϲػ���
} OperationMode;

// ��7 - ����Ũ�ȴ���������
typedef enum
{
    OxygenSensorType_NotInstalled = 1, // δ��װ
    OxygenSensorType_Analog,           // ģ��
    OxygenSensorType_Digital           // ����
} OxygenSensorType;

// ��8 - ͨ��ģʽ
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

// ��9 - ��·����
typedef enum
{
    TubeType_Endotracheal = 1, // Endo���ܹ�·
    TubeType_Tracheostomy      // ������ڹ�·
} TubeType;

// ��10 - ʪ��������
typedef enum
{
    HumidifierType_NotInstalled = 1, // δ��װ
    HumidifierType_Enabled,          // ����
    HumidifierType_HMEFilter         // HME������
} HumidifierType;

// ��11 - ©������
typedef enum
{
    LeakageCompensationType_Off = 1,  // �ر�
    LeakageCompensationType_Manual,   // �ֶ�
    LeakageCompensationType_Automatic // �Զ�
} LeakageCompensationType;

//__packed�ؼ�����ȷ���ṹ�岻�����κ����
typedef struct __packed
{
    uint16_t uIndex;     // ����������ţ���Ч��Χ0~35999
    int16_t s16P1;       // 10pa
    int16_t s16P2;       // 10pa
    int16_t s16P3;       // 10pa
    uint16_t u16F1;      // 10ml/����
    uint16_t u16F2;      // 10ml/����
    uint16_t u16F3;      // 10ml/����
    uint8_t uO2;         // 1%  ���齫uO2�Ƶ�ĩβ
    int16_t s16T1;       // 0.1���϶�
    int16_t s16T2;       // 0.1���϶�
    int16_t s16T3;       // 0.1���϶�
    uint32_t u16Motor;   //
    uint16_t u16MotorSV; // ͨ��PID�����Ŀ���ٶ�
    uint8_t uValve1;     // ����O2
    uint8_t uValve2;     // ��ŷ�P��
    uint8_t uValve3;     // ��ŷ�P��
    uint8_t uYDPValve;   // ��������

} struRtSample;
struct stru10HzSamplePerMinute
{
    uint8_t u8Year, u8Month, u8Day, u8Hour, u8Minute; // ��ǰ��¼ʱ��
    uint16_t u16P1[SAMPLES_PER_MINUTE], u16P2[SAMPLES_PER_MINUTE], u16P3[SAMPLES_PER_MINUTE];
    uint16_t u16F1[SAMPLES_PER_MINUTE], u16F2[SAMPLES_PER_MINUTE], u16F3[SAMPLES_PER_MINUTE];
};

struct struTarget
{
    uint8_t uVentilationMode;                        // ����ͨ��ģʽ
    uint16_t uInspirationTime;                       // Ŀ����������ʱ������λms��
    uint16_t uExpirationTime;                        // Ŀ���������ʱ������λms��
    uint16_t uPressureRiseTime;                      // ѹ������ʱ������λms��
    uint16_t uApnoeaTimeOut;                         // ������ͣʱ��
    uint16_t uMaximumPressure;                       // ͨ��ѹ����������
    uint16_t uInspirationDeltaPressure;              // ����ѹ��������, ���������ж�����������ʼ
    uint16_t uPositiveEndExpirationPressure;         // ������ѹͨ��ѹ����ѹ��������Ҫָ�꣩
    uint16_t uPressureSupportLowDeltaPressure;       // ����ѹ����������, ���������ж��Ƿ���������
    uint16_t uResuscitationSensitivityLevel;         // �������м���
    uint16_t uTidalVolume;                           // Ŀ�곱��������λml��
    uint16_t uMaximumPressurePCVR;                   // PCVR���ѹ��
    uint16_t uMinimumPressurePCVR;                   // PCVR��Сѹ��
    bool bIntrinsicPeepEnable;                       // �ڲ�PEEPʹ��
    uint16_t uPercentageO2;                          // ��Ũ�Ȱٷֱ�
    uint16_t uContinuousFlow;                        // ��������
    uint16_t uOxygenSupplyConcentration;             // ����ӦŨ��
    uint16_t uInspiratoryTriggerFlow;                // ����������
    uint16_t uExpiratoryFlowCycledTriggerPercentage; // ��������ѭ�������ٷֱ�
    bool bPCVREnable;                                // PCVRʹ��
    bool bNonInvasiveVentilationEnable;              // ������ʽͨ��ʹ��
    bool bPressureSupportLowEnable;                  // ��ѹ֧��ʹ��
    bool bInspiratoryFlowTriggerEnable;              // ������������ʹ��
    bool bLPOEnable;                                 // LPOʹ��
    bool bSighEnable;                                // ̾Ϣʹ��
    bool bTubeCompensationInspirationEnable;         // ��·��������ʹ��
    bool bP01Enable;                                 // P01ʹ��
    uint16_t uRecruitmentInspirationDeltaPressure;   // ��ļ����ѹ����
    uint16_t uRecruitmentPEEP;                       // ��ļPEEP
    uint16_t uRecruitmentNumberofBreaths;            // ��ļ��������
    bool bNebulizerEnable;                           // ����ʹ��
    uint16_t uSighDeltaPressure;                     // ̾Ϣѹ����
    uint16_t uSighDeltaTidalVolume;                  // ̾Ϣ��������
    uint16_t uSighBreathCount;                       // ̾Ϣ��������
    bool bInspiratoryHoldEnable;                     // ��������ʹ��
    bool bRecruitmentEnable;                         // ��ļʹ��
    bool bExpiratoryHoldEnable;                      // ��������ʹ��
    uint8_t uPatientType;                            // ��������
    uint8_t uPatientSensorPosition;                  // ���ߴ�����λ��
    uint8_t uExpirationValvePosition;                // ������λ��
    uint8_t uPatientSensorTemperature;               // ���ߴ������¶�����
    bool bCompensateCompressibleVolumeEnable;        // ��ѹ���������ʹ��
    uint8_t uTubeType;                               // ��·����
    uint8_t uTubeDiameter;                           // ��·ֱ��
    uint8_t TubeCompensationPercentage;              // ��·�����ٷֱ�
    bool bTubeCompensationExpirationEnable;          // ��·��������ʹ��
    uint16_t uNebulizerActiveTime;                   // �����ʱ��
    uint8_t uPatientSensorType;                      // ���ߴ���������
    uint8_t uGasStandard;                            // �����׼
    uint8_t uHumidifierType;                         // ��ʪ������
    uint8_t uSensitivitySpontaneousDetection;        // �Է��Լ��������
    uint16_t uMinuteVolume;                          // ����ͨ����
    uint16_t uInspiratoryFlow;                       // ��������
    uint8_t uInspiratorySpontaneousBreathingWindow;  // ��������������
    uint8_t uExpiratorySpontaneousBreathingWindow;   // ��������������
    bool bInspiratoryPressureTriggerEnable;          // ����ѹ������ʹ��
    uint16_t uInspiratoryTriggerDeltaPressure;       // ��������ѹ����
    int16_t sExpiratoryTriggerFlow;                  // ����������
    uint16_t uPressureSupportMaxStrokeTime;          // ѹ��֧������з�ʱ��
    uint16_t uInspiratoryHoldTime;                   // ��������ʱ��
    uint8_t uLeakageCompensationType;                // ©����������
    uint8_t uLeakageCompensationPercentage;          // ©�������ٷֱ�
    bool bPressureSupportHighEnable;                 // ��ѹ֧��ʹ��
    uint16_t uPressureSupportHighDeltaPressure;      // ��ѹ֧��ѹ����
    bool bApnoeaDetectionEnable;                     // ������ͣ���ʹ��
    uint16_t uRecruitmentInspirationTime;            // ��ļ����ʱ��
    uint16_t uRecruitmentExpirationTime;             // ��ļ����ʱ��
    uint16_t uRecruitmentPressureRiseTime;           // ��ļѹ������ʱ��
    uint16_t uNebulizerExternallyDeliveredFlow;      // �ⲿ��������
    uint8_t uNebulizerPosition;                      // ����λ��
    bool bPPSEnable;                                 // PPSʹ��
    uint8_t uPPSPercentageSupport;                   // PPS�ٷֱ�֧��
    uint16_t uPPSCompliance;                         // PPS˳Ӧ��
    uint16_t uPPSResistance;                         // PPS����
    uint16_t uExpiratoryFlowTriggerEnable;           // ������������ʹ��
    uint16_t uExpiratoryFlowCycledTriggerEnable;     // ��������ѭ������ʹ��
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
//                       ����λ��ͨ��Э��
//-----------------------------------------------------------------------

//! ��λ�����忨����������������������������������������
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

//! �忨����λ��������������������������������������
#define UP_FRAME_HEADER_BYTE_1 0xEB
#define UP_FRAME_HEADER_BYTE_2 0x91
#define UP_FRAME_HEADER_BYTE_3 0xAA

void upload_data_frame(uint8_t instruction_type, void *data, uint16_t data_length);
void upload_struRtSample(void);

//-----------------------------------------------------------------------
//                       ������־���
//-----------------------------------------------------------------------
void motor_pressure_flow(int min_speed, int max_speed);
void RTOS_motor_pressure_flow(int min_speed, int max_speed);

#endif