#ifndef _SOFTWARE_WANGJIAJUN
#define _SOFTWARE_WANGJIAJUN

#define GD32F450

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_gpio.h"
#include "gd32f4xx_timer.h"
#include "gd32f4xx_misc.h"
#include "gd32f4xx_dma.h"
#include "gd32f4xx_usart.h"
#include "gd32f4xx_i2c.h"
#include "gd32f4xx_exti.h"
#include "gd32f4xx_spi.h"
#include "gd32f4xx_adc.h"
#include "gd32f4xx_sdio.h"
#include "gd32f4xx_fmc.h"

#include "./sdcard.h"

// #define BOARD_VER_1
#define BOARD_VER_2

#define ON SET
#define OFF RESET

#define MAX_STR_SIZE 12 // ����ַ������ȣ����ǵ�32λ�������Ϊ10λ�����Ϸ��źͿ��ַ�

#define I2C0_OWN_ADDRESS7 0x72
#define I2C1_OWN_ADDRESS7 0x92

#define ZXP3010D_Address 0xDA
#define ZXP3010D_CMD 0x30

#define FS4301_Address 0xa0
#define FS4301_CMD 0xa1

#define ADC_CHANNEL_COUNT 6
#define MOTOR_FRAME_SIZE 6

// ȫ����������
//! ��Խ����ĵ�����
#define SAMPLE_RATE 10                        //! ָ����100
#define SAMPLES_PER_MINUTE (60 * SAMPLE_RATE) // ÿ���Ӳ������ݻ�������

struct struRtSample
{
    uint16_t uIndex;
    int16_t u16P1, u16P2, u16P3; // mbar
    uint16_t u16F1, u16F2, u16F3; // ml
    uint8_t uO2;                  // %
    uint8_t uT1, uT2, uT3;
    uint16_t uMotor;
    bool bValve1, bValve2, bValve3;
};

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

struct struRtMeasure
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
    uint16_t uMeanPressure, uMinimumPressure;
    uint16_t uP01Pressure;
    uint16_t uIntrinsicPeep;
    uint16_t uExpiratoryTidalVolumeSp, uExpiratoryTidalVolumeMand, uExpiratoryMinuteVolumeSp, uExpiratoryMinuteVolumeMand;
    uint16_t uRespiratoryRateSp, uRespiratoryRateMand, uStrokewiseFiO2, uLungResistance, uDynamicCompliance, uRSBI, uPressureTimeProduct, uLeakageVolume, uAmbientPressure;
    uint16_t uOxygenSupplyPressure, uGasInputTemperature, uGasOutputTemperature, uStandbyTime, uRunningTime, uPatientVolume, uPlateauPressure, uRealtimeFiO2;
    uint16_t uInspiratoryTidalVolumeMand, uInspiratoryMinuteVolumeMand, uInspiratoryTidalVolumeSp, uInspiratoryMinuteVolumeSp;
    uint16_t uLeakageFlow, uInspiratoryTime, uExpiratoryTime, uRCInspiratoryTime, uRCExpiratoryTime, uCompressibleVolume, uExpiratoryMinuteVolumeTotal, uRespiratoryRateTotal;
};

//! ���������д:
extern volatile uint8_t MOTOR_received_frame[MOTOR_FRAME_SIZE];
extern volatile uint16_t adc_values_454[ADC_CHANNEL_COUNT];
typedef struct
{
    uint8_t frame_header; // ֡ͷ
    uint16_t current_speed;
    int8_t motor_temperature; // ����¶�
    uint8_t fault_alarm;     // ���ϱ���״̬
    uint8_t checksum;        // У���
    uint8_t checksum_valid;  // У����Ƿ���Ч
} MotorStatus;
typedef struct
{
    bool p4_valve;    // P4������
    bool p5_valve;    // P5��ŷ�
    bool p6_valve;    // P6��ŷ�
    uint16_t pse540;      // PSE540
    uint8_t temperature; // �¶�(����)
    uint8_t oxygen;      // P16������Ũ�ȴ�����
} SensorData;
typedef struct
{
    uint32_t rising_edge_value0;  // ������ʱ�ļ�����ֵ
    uint32_t falling_edge_value0; // �½���ʱ�ļ�����ֵ
    float duty_cycle0;            // ����õ���ռ�ձ�

    uint32_t rising_edge_value1;  // ������ʱ�ļ�����ֵ
    uint32_t falling_edge_value1; // �½���ʱ�ļ�����ֵ
    float duty_cycle1;            // ����õ���ռ�ձ�

    uint32_t rising_edge_value2;  // ������ʱ�ļ�����ֵ
    uint32_t falling_edge_value2; // �½���ʱ�ļ�����ֵ
    float duty_cycle2;            // ����õ���ռ�ձ�
} pwm_capture_data_t;

//-----------------------------------------------------------------------
//                       ����λ��ͨ��Э��
//-----------------------------------------------------------------------
// ����ͨ��Э���е�֡ͷ��֡β
#define FRAME_HEADER_BYTE_1 0xEB
#define FRAME_HEADER_BYTE_2 0x90
#define FRAME_HEADER_BYTE_3 0xAA

// ����ָ�����
#define INSTRUCTION_TYPE_01 0x01
#define INSTRUCTION_TYPE_02 0x02

// ���崮��ͨ�ź���������������Ѿ����úô���
extern void send_data(uint8_t *data, uint16_t length);

// �������ݽṹ����ʾ���յ���ָ��
typedef struct {
    uint8_t instruction_type;
    uint8_t data[50];  // ����������ݳ���Ϊ50�ֽڣ�����ʵ���������
} ReceivedInstruction;

// ��������
void parse_received_data(uint8_t *received_data, uint16_t length);
void handle_instruction_type_01(ReceivedInstruction *instruction);
void handle_instruction_type_02(ReceivedInstruction *instruction);





//-----------------------------------------------------------------------
//                       ��ʼ������
//-----------------------------------------------------------------------
void init_454(void);

void RCU_init_454(void);
void NVIC_init_454(void);
void LED_init_454(void);
void YDP_init_454(void);
void Pulverizer_init_454(void);
void TIMER_init_454(void);
void USART0_init_454(void);
void USART1_init_454(void);
void USART2_init_454(void);
void I2C_init_454(void);
void SPI1_init_454(void);
void ADC2_DMA_init_454(void);
void ADC2_init_454(void);
void PWM_init_454(void);
void PWM_IN_init_454(void);
void SDIO_init_454(void);

// ���ߺ���
void delay_ms_454(uint32_t ms);
void delay_s_454(uint32_t seconds);
char *intToStr(int num);
char *floatToStr(float num, int afterpoint);
void usart_echo(uint32_t usart_periph);
// LED
void LED1(FlagStatus state);
void LED2(FlagStatus state);
void LED3(FlagStatus state);
// USART
uint8_t usart0_send_454(uint8_t *string, uint16_t count_size);
uint8_t usart0_receive_454(uint8_t *buffer, uint16_t buffer_size);
// I2C
uint8_t ZXP_Initial(uint32_t i2c_periph);
void ZXP_StartP(uint32_t i2c_periph);
void ZXP_StartT(uint32_t i2c_periph);
uint8_t ZXP_ConStatus(uint32_t i2c_periph);
int32_t ZXP_ResultP(uint32_t i2c_periph);
int32_t ZXP_ResultT(uint32_t i2c_periph);
void ZXP8_Caculate(int32_t up, int32_t ut, float *rp, float *rt);
void ZXP2_Caculate(int32_t up, int32_t ut, float *rp, float *rt);
void ZXP8_get_data_454(uint32_t i2c_periph, float *fTemp, float *fPress);
void ZXP2_get_data_454(uint32_t i2c_periph, float *fTemp, float *fPress);
void FS4301_get_data_454(uint32_t i2c_periph, uint16_t *flow_data);
uint32_t i2c_flag_check_timeout(uint32_t i2c_periph, i2c_flag_enum flag, FlagStatus expected_Status);
int i2c_master_receive(uint32_t i2c_periph, uint8_t *data, uint16_t length, uint16_t address);
int i2c_master_send(uint32_t i2c_periph, uint8_t *data, uint16_t length, uint16_t address);
void I2C_Scan(uint32_t i2c_periph);
// ѹ����ȡ
int16_t P10_get(void);
int16_t P11_get(void);
int16_t P12_get(void);
// ������ȡ
uint16_t P13_get(void);
uint16_t P14_get(void);
uint16_t P15_get(void);
// SPI
void MAX31865_CsOn(uint32_t cs_pin);
void MAX31865_CsOff(uint32_t cs_pin);
uint8_t SPI1_Transfer(uint32_t cs_pin, uint8_t data);
void MAX31865_Spi_WriteByte(uint32_t cs_pin, uint8_t data);
uint8_t MAX31865_Spi_ReadByte(uint32_t cs_pin);
void MAX31865_bufWrite(uint32_t cs_pin, uint8_t addr, uint8_t value);
uint8_t MAX31865_bufRead(uint32_t cs_pin, uint8_t addr);
void MAX31865_HWInit(uint32_t cs_pin);
uint16_t MAX31865_TempGet_454(uint32_t cs_pin);
FlagStatus drdy1_status(void);
FlagStatus drdy2_status(void);
// �¶Ȼ�ȡ
int16_t P7_get(void);
int16_t P9_get(void);
// ADC
float adc_to_voltage(uint16_t adc_value);

// PWM
void P4_PWM_set(uint32_t pulse);
void P5_PWM_set(uint32_t pulse);
void P6_PWM_set(uint32_t pulse);
// ѹ�緧Ƭ
void YDP_control(FlagStatus on);
FlagStatus YDP_status_get(void);
// ��������
void Pulverizer_control(FlagStatus on);
FlagStatus Pulverizer_status_get(void);

// ���
void motor_control(uint16_t speed);
void motor_speed_percent(uint8_t percent);

// ������־���
void motor_pressure_flow(int min_speed, int max_speed);
void RTOS_motor_pressure_flow(int min_speed, int max_speed);
//У׼
void align_data(void);

// SD��
void SD_init_454(void);
sd_error_enum sd_io_init(void);
void card_info_get(void);

#endif