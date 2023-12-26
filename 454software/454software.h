#ifndef _SOFTWARE_WANGJIAJUN
#define _SOFTWARE_WANGJIAJUN

#define GD32F450

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
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

// ȫ������
extern volatile uint8_t MOTOR_received_frame[MOTOR_FRAME_SIZE];
extern volatile uint16_t adc_values_454[ADC_CHANNEL_COUNT];
typedef struct
{
    uint8_t frame_header; // ֡ͷ
    // uint16_t current_speed;
    //! ʹ��float�������
    float current_speed; // ��ǰת��
    // int8_t motor_temperature;
    float motor_temperature; // ����¶�
    uint8_t fault_alarm;     // ���ϱ���״̬
    uint8_t checksum;        // У���
    uint8_t checksum_valid;  // У����Ƿ���Ч
} MotorStatus;
typedef struct
{
    float p4_valve;    // P4������
    float p5_valve;    // P5��ŷ�
    float p6_valve;    // P6��ŷ�
    float pse540;      // PSE540
    float temperature; // �¶�(����)
    float oxygen;      // P16������Ũ�ȴ�����
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

// ��ʼ������
void init_454(void);

void RCU_init_454(void);
void NVIC_init_454(void);
void LED_init_454(void);
void YDP_init_454(void);
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

// ���ߺ���
void delay_ms_454(uint32_t ms);
void delay_s_454(uint32_t seconds);
char *intToStr(int num);
char *floatToStr(float num, int afterpoint);
void usart_echo(uint32_t usart_periph);

// USART
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
void FS4301_get_data_454(uint32_t i2c_periph, float *flow_data);
uint32_t i2c_flag_check_timeout(uint32_t i2c_periph, i2c_flag_enum flag, FlagStatus expected_Status);
int i2c_master_receive(uint32_t i2c_periph, uint8_t *data, uint16_t length, uint16_t address);
int i2c_master_send(uint32_t i2c_periph, uint8_t *data, uint16_t length, uint16_t address);
void I2C_Scan(uint32_t i2c_periph);
// ѹ����ȡ
float P10_get(void);
float P11_get(void);
float P12_get(void);
// ������ȡ
float P13_get(void);
float P14_get(void);
float P15_get(void);
// SPI
void MAX31865_CsOn(uint32_t cs_pin);
void MAX31865_CsOff(uint32_t cs_pin);
uint8_t SPI1_Transfer(uint32_t cs_pin, uint8_t data);
void MAX31865_Spi_WriteByte(uint32_t cs_pin, uint8_t data);
uint8_t MAX31865_Spi_ReadByte(uint32_t cs_pin);
void MAX31865_bufWrite(uint32_t cs_pin, uint8_t addr, uint8_t value);
uint8_t MAX31865_bufRead(uint32_t cs_pin, uint8_t addr);
void MAX31865_HWInit(uint32_t cs_pin);
int16_t MAX31865_TempGet_454(uint32_t cs_pin);
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

// ���
void motor_control(uint16_t speed);
void motor_speed_percent(uint8_t percent);

// ������־���
void motor_pressure_flow(int min_speed, int max_speed);
#endif