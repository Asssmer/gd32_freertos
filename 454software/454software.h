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

#define MAX_STR_SIZE 12 // 最大字符串长度，考虑到32位整数最大为10位，加上符号和空字符

#define I2C0_OWN_ADDRESS7 0x72
#define I2C1_OWN_ADDRESS7 0x92

#define ZXP3010D_Address 0xDA
#define ZXP3010D_CMD 0x30

#define FS4301_Address 0xa0
#define FS4301_CMD 0xa1

#define ADC_CHANNEL_COUNT 6
#define MOTOR_FRAME_SIZE 6

// 全局数据声明
//! 余辉教授文档需求
#define SAMPLE_RATE 10                        //! 指定是100
#define SAMPLES_PER_MINUTE (60 * SAMPLE_RATE) // 每分钟采样数据缓存数量

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

//! 最初驱动编写:
extern volatile uint8_t MOTOR_received_frame[MOTOR_FRAME_SIZE];
extern volatile uint16_t adc_values_454[ADC_CHANNEL_COUNT];
typedef struct
{
    uint8_t frame_header; // 帧头
    uint16_t current_speed;
    int8_t motor_temperature; // 电机温度
    uint8_t fault_alarm;     // 故障报警状态
    uint8_t checksum;        // 校验和
    uint8_t checksum_valid;  // 校验和是否有效
} MotorStatus;
typedef struct
{
    bool p4_valve;    // P4比例阀
    bool p5_valve;    // P5电磁阀
    bool p6_valve;    // P6电磁阀
    uint16_t pse540;      // PSE540
    uint8_t temperature; // 温度(备用)
    uint8_t oxygen;      // P16辅助氧浓度传感器
} SensorData;
typedef struct
{
    uint32_t rising_edge_value0;  // 上升沿时的计数器值
    uint32_t falling_edge_value0; // 下降沿时的计数器值
    float duty_cycle0;            // 计算得到的占空比

    uint32_t rising_edge_value1;  // 上升沿时的计数器值
    uint32_t falling_edge_value1; // 下降沿时的计数器值
    float duty_cycle1;            // 计算得到的占空比

    uint32_t rising_edge_value2;  // 上升沿时的计数器值
    uint32_t falling_edge_value2; // 下降沿时的计数器值
    float duty_cycle2;            // 计算得到的占空比
} pwm_capture_data_t;

//-----------------------------------------------------------------------
//                       与上位机通信协议
//-----------------------------------------------------------------------
// 定义通信协议中的帧头和帧尾
#define FRAME_HEADER_BYTE_1 0xEB
#define FRAME_HEADER_BYTE_2 0x90
#define FRAME_HEADER_BYTE_3 0xAA

// 定义指令类别
#define INSTRUCTION_TYPE_01 0x01
#define INSTRUCTION_TYPE_02 0x02

// 定义串口通信函数，这里假设你已经配置好串口
extern void send_data(uint8_t *data, uint16_t length);

// 定义数据结构，表示接收到的指令
typedef struct {
    uint8_t instruction_type;
    uint8_t data[50];  // 假设最大数据长度为50字节，根据实际情况调整
} ReceivedInstruction;

// 函数声明
void parse_received_data(uint8_t *received_data, uint16_t length);
void handle_instruction_type_01(ReceivedInstruction *instruction);
void handle_instruction_type_02(ReceivedInstruction *instruction);





//-----------------------------------------------------------------------
//                       初始化函数
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

// 工具函数
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
// 压力获取
int16_t P10_get(void);
int16_t P11_get(void);
int16_t P12_get(void);
// 流量获取
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
// 温度获取
int16_t P7_get(void);
int16_t P9_get(void);
// ADC
float adc_to_voltage(uint16_t adc_value);

// PWM
void P4_PWM_set(uint32_t pulse);
void P5_PWM_set(uint32_t pulse);
void P6_PWM_set(uint32_t pulse);
// 压电阀片
void YDP_control(FlagStatus on);
FlagStatus YDP_status_get(void);
// 雾化器控制
void Pulverizer_control(FlagStatus on);
FlagStatus Pulverizer_status_get(void);

// 电机
void motor_control(uint16_t speed);
void motor_speed_percent(uint8_t percent);

// 测试日志输出
void motor_pressure_flow(int min_speed, int max_speed);
void RTOS_motor_pressure_flow(int min_speed, int max_speed);
//校准
void align_data(void);

// SD卡
void SD_init_454(void);
sd_error_enum sd_io_init(void);
void card_info_get(void);

#endif